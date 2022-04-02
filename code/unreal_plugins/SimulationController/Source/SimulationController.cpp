// Copyright Epic Games, Inc. All Rights Reserved.

#include "SimulationController.h"

#include <future>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Engine/Engine.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include "GameFramework/GameModeBase.h"
#include <Kismet/GameplayStatics.h>

#include "AgentController.h"
#include "Assert.h"
#include "Box.h"
#include "Config.h"
#include "RpcServer.h"
#include "SphereAgentController.h"

// different possible frame states for thread synchronization
enum class FrameState : uint8_t
{
    Idle,
    RequestPreTick,
    ExecutingPreTick,
    ExecutingTick,
    ExecutingPostTick
};

void SimulationController::StartupModule()
{
    // Initialize config system
    Config::initialize();

    // required to add ActorSpawnedEventHandler
    post_world_initialization_delegate_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &SimulationController::postWorldInitializationEventHandler);

    // required to reset any custom logic during world cleanup
    world_cleanup_delegate_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &SimulationController::worldCleanupEventHandler);

    // required for adding thread synchronization logic
    begin_frame_delegate_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &SimulationController::beginFrameEventHandler);
    end_frame_delegate_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &SimulationController::endFrameEventHandler);

    // initialize variables used in thread synchronization
    end_frame_started_executing_promise_ = std::promise<void>();
    end_frame_started_executing_future_ = end_frame_started_executing_promise_.get_future();

    end_frame_finished_executing_promise_ = std::promise<void>();
    end_frame_finished_executing_future_ = end_frame_finished_executing_promise_.get_future();
    
    frame_state_ = FrameState::Idle;
}

void SimulationController::ShutdownModule()
{
    // If this module is unloaded in the middle of simulation for some reason, raise an error because we do not support this and we want to know when this happens.
    // We expect worldCleanUpEvenHandler() to be called before ShutdownModule(). 
    ASSERT(!world_begin_play_delegate_handle_.IsValid());

    // Remove event handlers used by this module
    FCoreDelegates::OnEndFrame.Remove(end_frame_delegate_handle_);
    end_frame_delegate_handle_.Reset();

    FCoreDelegates::OnBeginFrame.Remove(begin_frame_delegate_handle_);
    begin_frame_delegate_handle_.Reset();

    FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_delegate_handle_);
    world_cleanup_delegate_handle_.Reset();
    
    FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_delegate_handle_);
    post_world_initialization_delegate_handle_.Reset();

    // Terminate config system as we will not use it anymore
    Config::terminate();
}

void SimulationController::postWorldInitializationEventHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    ASSERT(world);

    if (world->IsGameWorld()) {
        // Check if world_ is valid, and if it is, we do not support mulitple Game worlds and we need to know about this. There should only be one Game World..
        ASSERT(!world_);

        // Cache local reference of World instance as this is required in other parts of this class.
        world_ = world;
        
        // Required to assign an AgentController based on config param
        world_begin_play_delegate_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &SimulationController::worldBeginPlayEventHandler);
    }
}

void SimulationController::worldBeginPlayEventHandler()
{
    // Set few console commands for syncing Game Thread (GT) and RHI thread.
    // For more information on GTSyncType, see http://docs.unrealengine.com/en-US/SharingAndReleasing/LowLatencyFrameSyncing/index.html.
    GEngine->Exec(world_, TEXT("r.GTSyncType 1"));
    GEngine->Exec(world_, TEXT("r.OneFrameThreadLag 0"));

    // Set fixed simulation step time in seconds
    FApp::SetBenchmarking(true);
    FApp::SetFixedDeltaTime(Config::getValue<double>({"SIMULATION_CONTROLLER", "SIMULATION_STEP_TIME_SECONDS"}));

    // @TODO: Read and set random seed value
    // Seed = Config::getValue<int>({"INTERIORSIM", "RANDOM_SEED"});
    // SetRandomStreamSeed(Seed); // @TODO: complete this

    // pause gameplay
    UGameplayStatics::SetGamePaused(world_, true);

    // @TODO: Read config to decide which type of AgentController to create
    if(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "AGENT_CONTROLLER_NAME"}) == "SphereAgentController") {
        agent_controller_ = std::make_unique<SphereAgentController>(world_);
    }
    else if(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "AGENT_CONTROLLER_NAME"}) == "DebugAgentController") {
        agent_controller_ = std::make_unique<DebugAgentController>(world_);
    }
    else {
        ASSERT(false);
    }

    // @TODO: Add separate Task class (to compute reward)

    
    // config values required for rpc communication
    const std::string hostname = Config::getValue<std::string>({"INTERIORSIM", "IP"});
    const int port = Config::getValue<int>({"INTERIORSIM", "PORT"});

    rpc_server_ = std::make_unique<RpcServer>(hostname, port);
    ASSERT(rpc_server_);
    
    bindFunctionsToRpcServer();
    
    rpc_server_->launchWorkerThreads(1u);

    is_world_begin_play_executed_ = true;
}

void SimulationController::worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    ASSERT(world);

    if (world->IsGameWorld()) {
        ASSERT(world_);

        // OnWorldCleanUp is called for all worlds. rpc_server_ and agent_controller_ is created only when a Game World's begin play event is launched.
        if(is_world_begin_play_executed_) {
            ASSERT(rpc_server_);
            rpc_server_->stop(); // stop the RPC server as we will no longer service client requests

            ASSERT(agent_controller_);
            agent_controller_.reset(nullptr);
        }

        // Remove event handlers bound to this world before world gets cleaned up
        world_->OnWorldBeginPlay.Remove(world_begin_play_delegate_handle_);
        world_begin_play_delegate_handle_.Reset();

        // Clear local cache
        world_ = nullptr;
    }
}

void SimulationController::beginFrameEventHandler()
{
    // if beginTick() has indicated (via RequestPreTick framestate) that we should execute a frame of work
    if (frame_state_ == FrameState::RequestPreTick) {

        // update frame state
        frame_state_ = FrameState::ExecutingPreTick;

        // unpause the game
        UGameplayStatics::SetGamePaused(world_, false);

        // execute all pre-tick sync work, wait here for tick() to reset work guard
        rpc_server_->RunSync();
    
        // reinitialze the io_context and work guard after runSync to prepare for next run
        rpc_server_->reinitializeIOContextAndWorkGuard();

        // update frame state so that end frame can execute
        frame_state_ = FrameState::ExecutingTick;
    }
}

void SimulationController::endFrameEventHandler()
{
    if (frame_state_ == FrameState::ExecutingTick) {

        // update frame state so that endTick() can execute successfully
        frame_state_ = FrameState::ExecutingPostTick;

        // allow tick() to finish executing
        end_frame_started_executing_promise_.set_value();

        // execute all post-tick sync work, wait here for endTick() to reset work guard
        rpc_server_->RunSync();

        // reinitialze the io_context and work guard after runSync to prepare for next run
        rpc_server_->reinitializeIOContextAndWorkGuard();

        // pause the game
        UGameplayStatics::SetGamePaused(world_, true);

        // update frame state so that beginTick() can execute successfully
        frame_state_ = FrameState::Idle;

        // notify that end frame has finished executing so that endTick() can finish executing
        end_frame_finished_executing_promise_.set_value();
    }
}

void SimulationController::bindFunctionsToRpcServer()
{
    rpc_server_->bindAsync("ping", []() -> std::string {
        return "received ping";
    });

    rpc_server_->bindAsync("close", []() -> void {
        constexpr bool immediate_shutdown = false;
        FGenericPlatformMisc::RequestExit(immediate_shutdown);
    });

    rpc_server_->bindAsync("isPaused", [this]() -> bool {
        return world_->GetAuthGameMode()->IsPaused();
    });

    rpc_server_->bindAsync("getEndianness", []() -> uint8_t {
        enum class Endianness : uint8_t
        {
            LittleEndian = 0,
            BigEndian = 1,
        };        
        uint32_t Num = 0x01020304;
        return (reinterpret_cast<const char*>(&Num)[3] == 1) ? static_cast<uint8_t>(Endianness::LittleEndian) : static_cast<uint8_t>(Endianness::BigEndian);
    });

    rpc_server_->bindAsync("beginTick", [this]() -> void {
        ASSERT(frame_state_ == FrameState::Idle);

        // reinitialize end_frame_started_executing promise and future
        end_frame_started_executing_promise_ = std::promise<void>();
        end_frame_started_executing_future_ = end_frame_started_executing_promise_.get_future();

        // reinitialize end_frame_finished_executing promise and future
        end_frame_finished_executing_promise_ = std::promise<void>();
        end_frame_finished_executing_future_ = end_frame_finished_executing_promise_.get_future();

        // indicate that we want the game thread to execute one frame of work
        frame_state_ = FrameState::RequestPreTick;
    });

    rpc_server_->bindAsync("tick", [this]() -> void {
        ASSERT((frame_state_ == FrameState::ExecutingPreTick) || (frame_state_ == FrameState::RequestPreTick));

        // indicate that we want the game thread to stop blocking in begin_frame()
        rpc_server_->resetWorkGuard();

        // wait here until the game thread has started executing end_frame()
        end_frame_started_executing_future_.wait();

        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
    });

    rpc_server_->bindAsync("endTick", [this]() -> void {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);

        // indicate that we want the game thread to stop blocking in end_frame()
        rpc_server_->resetWorkGuard();

        // wait here until the game thread has finished executing end_frame()
        end_frame_finished_executing_future_.wait();

        ASSERT(frame_state_ == FrameState::Idle);
    });

    rpc_server_->bindAsync("getObservationSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(agent_controller_);
        return agent_controller_->getObservationSpace();
    });

    rpc_server_->bindAsync("getActionSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(agent_controller_);
        return agent_controller_->getActionSpace();
    });

    rpc_server_->bindSync("getObservation", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        ASSERT(agent_controller_);
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        return agent_controller_->getObservation();
    });

    rpc_server_->bindSync("applyAction", [this](std::map<std::string, std::vector<float>> action) -> void {
        ASSERT(agent_controller_);
        ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        agent_controller_->applyAction(action);
    });
}

IMPLEMENT_MODULE(SimulationController, SimulationController)
