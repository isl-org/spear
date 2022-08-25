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
#include <GameFramework/GameModeBase.h>
#include <Kismet/GameplayStatics.h>

#include "AgentController.h"
#include "Assert.h"
#include "Box.h"
#include "Config.h"
#include "ImitationLearningTask.h"
#include "NullTask.h"
#include "OpenBotAgentController.h"
#include "PointGoalNavTask.h"
#include "Rpclib.h"
#include "RpcServer.h"
#include "SphereAgentController.h"
#include "Task.h"
#include "Visualizer.h"

// Different possible frame states for thread synchronization
enum class FrameState : uint8_t
{
    Idle,
    RequestPreTick,
    ExecutingPreTick,
    ExecutingTick,
    ExecutingPostTick
};

// enum values should match values in python module (env.py)
enum class Endianness : uint8_t
{
    LittleEndian = 0,
    BigEndian = 1,
};
MSGPACK_ADD_ENUM(Endianness);

void SimulationController::StartupModule()
{
    ASSERT(FModuleManager::Get().IsModuleLoaded(TEXT("CoreUtils")));

    // Required to add ActorSpawnedEventHandler
    post_world_initialization_delegate_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &SimulationController::postWorldInitializationEventHandler);

    // Required to reset any custom logic during world cleanup
    world_cleanup_delegate_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &SimulationController::worldCleanupEventHandler);

    // Required for adding thread synchronization logic
    begin_frame_delegate_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &SimulationController::beginFrameEventHandler);
    end_frame_delegate_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &SimulationController::endFrameEventHandler);
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

    // Pause gameplay
    UGameplayStatics::SetGamePaused(world_, true);

    // Read config to decide which type of Task class to create
    if(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "TASK_NAME"}) == "PointGoalNavigationTask") {
        task_ = std::make_unique<PointGoalNavTask>(world_);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "TASK_NAME"}) == "ImitationLearningTask") {
        task_ = std::make_unique<ImitationLearningTask>(world_);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "TASK_NAME"}) == "NullTask") {
        task_ = std::make_unique<NullTask>();
    } else {
        ASSERT(false);
    }
    ASSERT(task_);

    // Read config to decide which type of AgentController to create
    if(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "AGENT_CONTROLLER_NAME"}) == "SphereAgentController") {
        agent_controller_ = std::make_unique<SphereAgentController>(world_);
    } else if(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "AGENT_CONTROLLER_NAME"}) == "OpenBotAgentController") {
        agent_controller_ = std::make_unique<OpenBotAgentController>(world_);
    } else if(Config::getValue<std::string>({"SIMULATION_CONTROLLER", "AGENT_CONTROLLER_NAME"}) == "DebugAgentController") {
        agent_controller_ = std::make_unique<DebugAgentController>(world_);
    } else {
        ASSERT(false);
    }
    ASSERT(agent_controller_);

    // Create a visualizer that is responsible for the camera view
    visualizer_ = std::make_unique<Visualizer>(world_);
    ASSERT(visualizer_);

    // Initialize frame state used for thread synchronization
    frame_state_ = FrameState::Idle;

<<<<<<< HEAD
    // Config values required for rpc communication
    const std::string hostname = Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IP"});
    const int port = Config::getValue<int>({"SIMULATION_CONTROLLER", "PORT"});
=======
    // config values required for rpc communication
    const auto hostname = Config::getValue<std::string>({"SIMULATION_CONTROLLER", "IP"});
    const auto port = Config::getValue<int>({"SIMULATION_CONTROLLER", "PORT"});
>>>>>>> origin/main

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
            rpc_server_ = nullptr;

            ASSERT(visualizer_);
            visualizer_ = nullptr;

            ASSERT(task_);
            task_ = nullptr;

            ASSERT(agent_controller_);
            agent_controller_ = nullptr;
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
    // If beginTick() has indicated (via RequestPreTick framestate) that we should execute a frame of work
    if (frame_state_ == FrameState::RequestPreTick) {

        // Update local state
        frame_state_ = FrameState::ExecutingPreTick;

        // Unpause the game
        UGameplayStatics::SetGamePaused(world_, false);

        // Execute all pre-tick sync work, wait here for tick() to reset work guard
        rpc_server_->runSync();

        // Execute pre-tick work inside the task
        task_->beginFrame();

        // Update local state
        frame_state_ = FrameState::ExecutingTick;
    }
}

void SimulationController::endFrameEventHandler()
{
    // If beginFrameEventHandler() has indicated that we are currently executing a frame of work
    if (frame_state_ == FrameState::ExecutingTick) {

        // Update local state
        frame_state_ = FrameState::ExecutingPostTick;

        // Execute post-tick work inside the task
        task_->endFrame();

        // Allow tick() to finish executing
        end_frame_started_executing_promise_.set_value();

        // Execute all post-tick sync work, wait here for endTick() to reset work guard
        rpc_server_->runSync();

        // Pause the game
        UGameplayStatics::SetGamePaused(world_, true);

        // Update local state
        frame_state_ = FrameState::Idle;

        // Allow endTick() to finish executing
        end_frame_finished_executing_promise_.set_value();
    }
}

void SimulationController::bindFunctionsToRpcServer()
{
    rpc_server_->bindAsync("ping", []() -> std::string {
        return "SimulationController received a call to ping()...";
    });

    rpc_server_->bindAsync("close", []() -> void {
        constexpr auto immediate_shutdown = false;
        FGenericPlatformMisc::RequestExit(immediate_shutdown);
    });

    rpc_server_->bindAsync("getEndianness", []() -> Endianness {
        uint32_t dummy = 0x01020304;
        return (reinterpret_cast<const char*>(&dummy)[3] == 1) ? Endianness::LittleEndian : Endianness::BigEndian;
    });

    rpc_server_->bindAsync("beginTick", [this]() -> void {
        ASSERT(frame_state_ == FrameState::Idle);

        // Reinitialize end_frame_started_executing promise and future
        end_frame_started_executing_promise_ = std::promise<void>();
        end_frame_started_executing_future_ = end_frame_started_executing_promise_.get_future();

        // Reinitialize end_frame_finished_executing promise and future
        end_frame_finished_executing_promise_ = std::promise<void>();
        end_frame_finished_executing_future_ = end_frame_finished_executing_promise_.get_future();

        // Indicate that we want the game thread to execute one frame of work
        frame_state_ = FrameState::RequestPreTick;
    });

    rpc_server_->bindAsync("tick", [this]() -> void {
        ASSERT((frame_state_ == FrameState::ExecutingPreTick) || (frame_state_ == FrameState::RequestPreTick));

        // Indicate that we want the game thread to stop blocking in beginFrame()
        rpc_server_->unblockRunSyncWhenFinishedExecuting();

        // Wait here until the game thread has started executing endFrame()
        end_frame_started_executing_future_.wait();

        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
    });

    rpc_server_->bindAsync("endTick", [this]() -> void {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);

        // Indicate that we want the game thread to stop blocking in endFrame()
        rpc_server_->unblockRunSyncWhenFinishedExecuting();

        // Wait here until the game thread has finished executing endFrame()
        end_frame_finished_executing_future_.wait();

        ASSERT(frame_state_ == FrameState::Idle);
    });

    rpc_server_->bindAsync("getActionSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(agent_controller_);
        return agent_controller_->getActionSpace();
    });

    rpc_server_->bindAsync("getObservationSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(agent_controller_);
        return agent_controller_->getObservationSpace();
    });

    rpc_server_->bindAsync("getTaskStepInfoSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(task_);
        return task_->getStepInfoSpace();
    });

    rpc_server_->bindAsync("getAgentControllerStepInfoSpace", [this]() -> std::map<std::string, Box> {
        ASSERT(agent_controller_);
        return agent_controller_->getStepInfoSpace();
    });

    rpc_server_->bindSync("applyAction", [this](std::map<std::string, std::vector<float>> action) -> void {
        ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        ASSERT(agent_controller_);
        agent_controller_->applyAction(action);
    });

    rpc_server_->bindSync("getObservation", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(agent_controller_);
        return agent_controller_->getObservation();
    });

    rpc_server_->bindSync("getReward", [this]() -> float {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(task_);
        return task_->getReward();
    });

    rpc_server_->bindSync("isEpisodeDone", [this]() -> bool {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(task_);
        return task_->isEpisodeDone();
    });

    rpc_server_->bindSync("getTaskStepInfo", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(task_);
        return task_->getStepInfo();
    });

    rpc_server_->bindSync("getAgentControllerStepInfo", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(task_);
        return agent_controller_->getStepInfo();
    });

    rpc_server_->bindSync("reset", [this]() -> void {
        ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        ASSERT(task_);
        ASSERT(agent_controller_);
        task_->reset();
        agent_controller_->reset();
    });

    rpc_server_->bindSync("isReady", [this]() -> bool{
        ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        ASSERT(task_);
        ASSERT(agent_controller_);
        return task_->isReady() && agent_controller_->isReady();
    });
}

IMPLEMENT_MODULE(SimulationController, SimulationController)
