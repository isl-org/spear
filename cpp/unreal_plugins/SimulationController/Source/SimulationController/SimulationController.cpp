//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/SimulationController.h"

#include <stdint.h> // uint8_t, uint32_t

#include <atomic>
#include <future> // std::promise
#include <map>
#include <memory> // std::make_unique, std::unique_ptr
#include <string>
#include <vector>

#include <Engine/Engine.h>         // GEngine
#include <Engine/World.h>          // UWorld
#include <Kismet/GameplayStatics.h>
#include <Misc/App.h>
#include <Misc/CoreDelegates.h>
#include <Modules/ModuleManager.h> // IMPLEMENT_MODULE
#include <PhysicsEngine/PhysicsSettings.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/Agent.h"
#include "SimulationController/CameraAgent.h"
#include "SimulationController/ClassRegistrationUtils.h"
#include "SimulationController/ImitationLearningTask.h"
#include "SimulationController/NavMesh.h"
#include "SimulationController/NullTask.h"
#include "SimulationController/RpcServer.h"
#include "SimulationController/Scene.h"
#include "SimulationController/SphereAgent.h"
#include "SimulationController/Task.h"
#include "SimulationController/UrdfRobotAgent.h"
#include "SimulationController/Visualizer.h"
#include "SimulationController/VehicleAgent.h"

// Different possible frame states for thread synchronization
enum class FrameState
{
    Idle,
    RequestPreTick,
    ExecutingPreTick,
    ExecutingTick,
    ExecutingPostTick,
};

void SimulationController::StartupModule()
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(Unreal::toFName("CoreUtils")));
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(Unreal::toFName("UrdfRobot")));
    SP_ASSERT(FModuleManager::Get().IsModuleLoaded(Unreal::toFName("Vehicle")));

    if (!Config::s_initialized_) {
        return;
    }

    post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &SimulationController::postWorldInitializationEventHandler);
    world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &SimulationController::worldCleanupEventHandler);
    begin_frame_handle_ = FCoreDelegates::OnBeginFrame.AddRaw(this, &SimulationController::beginFrameEventHandler);
    end_frame_handle_ = FCoreDelegates::OnEndFrame.AddRaw(this, &SimulationController::endFrameEventHandler);
}

void SimulationController::ShutdownModule()
{
    SP_LOG_CURRENT_FUNCTION();

    if (!Config::s_initialized_) {
        return;
    }

    // If this module is unloaded in the middle of simulation for some reason, raise an error.
    // We expect worldCleanUpEvenHandler(...) to be called before ShutdownModule().
    SP_ASSERT(!world_begin_play_handle_.IsValid());

    FCoreDelegates::OnEndFrame.Remove(end_frame_handle_);
    FCoreDelegates::OnBeginFrame.Remove(begin_frame_handle_);
    FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
    FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

    end_frame_handle_.Reset();
    begin_frame_handle_.Reset();
    world_cleanup_handle_.Reset();
    post_world_initialization_handle_.Reset();
}

void SimulationController::postWorldInitializationEventHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    #if WITH_EDITOR // defined in an auto-generated header
        bool world_is_ready = world->IsGameWorld();
    #else
        bool world_is_ready = world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world);
    #endif

    if (world_is_ready) {

        auto scene_id = Config::get<std::string>("SIMULATION_CONTROLLER.SCENE_ID");
        auto map_id = Config::get<std::string>("SIMULATION_CONTROLLER.MAP_ID");

        std::string desired_world_path_name;
        std::string desired_level_name;
        if (scene_id != "") {
            if (map_id == "") {
                map_id = scene_id;
            }
            desired_world_path_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id + "." + map_id;
            desired_level_name      = "/Game/Scenes/" + scene_id + "/Maps/" + map_id;            
        }

        // if the current world is not the desired one, open the desired one
        bool open_level = desired_world_path_name != "" && desired_world_path_name != Unreal::toStdString(world->GetPathName());

        SP_LOG("scene_id:                ", scene_id);
        SP_LOG("map_id:                  ", map_id);
        SP_LOG("desired_world_path_name: ", desired_world_path_name);
        SP_LOG("desired_level_name:      ", desired_level_name);
        SP_LOG("world->GetPathName():    ", Unreal::toStdString(world->GetPathName()));
        SP_LOG("open_level:              ", open_level);

        if (open_level) {
            SP_LOG("Opening level: ", desired_level_name);

            // if we're at this line of code and OpenLevel is already pending, it means we failed
            SP_ASSERT(!open_level_pending_);

            UGameplayStatics::OpenLevel(world, Unreal::toFName(desired_level_name));
            open_level_pending_ = true;

        } else {
            open_level_pending_ = false;

            // we expect worldCleanupEventHandler(...) to be called before a new world is created
            SP_ASSERT(!world_);

            // cache local reference to the UWorld
            world_ = world;

            // We need to defer initializing this handler until after we have a valid world_ pointer,
            // and we defer the rest of our initialization code until the OnWorldBeginPlay event. We
            // wrap this code in an if block to enable interactive navigation mode, which will potentially
            // need to load a new map via the config system, but should not initialize the rest of our
            // code.
            if (Config::get<std::string>("SIMULATION_CONTROLLER.INTERACTION_MODE") == "programmatic") {
                world_begin_play_handle_ = world_->OnWorldBeginPlay.AddRaw(this, &SimulationController::worldBeginPlayEventHandler);

                // The component hierarchy that is setup in Unreal Editor (as displayed in the outliner window),
                // is modified when the game begins. Hence, this may cause issues in using GetAttachChildren()
                // on components because, some components will no longer be attached to their parents as seen in
                // the outliner. Instead, they would be moved to the same level/depth as the RootComponent.
                // We see this behavior on components whose SimulatePhysics=True. These components and their
                // children are moved to the same level/depth as the RootComponent. The reason could be that
                // when simulate_physics is true, these components no longer have to follow the parent, instead
                // they can move independently.
                //
                // Since we need to disable physics on all components anyway (physics is handled by MuJoCo), we will
                // disable it during PostWorldInitialization and not in WorldBeginPlay because the component hierarchy
                // would already be updated when the WorldBeginPlay is called. This way, the component hierarchy is
                // maintained.
                //
                // Also, we do this only in programmatic mode because we do not want to alter the SimulatePhysics
                // state while in Unreal Editor as our MuJoCo export code relies on this.

                std::map<std::string, AActor*> actors_name_ref_map_ = Unreal::findActorsByTagAllAsMap(world_, {});
                SP_ASSERT(!actors_name_ref_map_.empty());

                for (auto& element : actors_name_ref_map_) {
                    TArray<UPrimitiveComponent*> primitive_components;
                    element.second->GetComponents<UPrimitiveComponent*>(primitive_components);
                    for (auto& primitive_component : primitive_components) {
                        primitive_component->SetSimulatePhysics(false);
                    }
                }
            }
        }
    }
}

void SimulationController::worldBeginPlayEventHandler()
{
    SP_LOG_CURRENT_FUNCTION();

    // execute optional console commands from python client
    for (auto& command : Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.CUSTOM_UNREAL_CONSOLE_COMMANDS")) {
        GEngine->Exec(world_, *Unreal::toFString(command));
    }

    // set physics parameters
    UPhysicsSettings* physics_settings = UPhysicsSettings::Get();
    physics_settings->bEnableEnhancedDeterminism = Config::get<bool>("SIMULATION_CONTROLLER.PHYSICS.ENABLE_ENHANCED_DETERMINISM");
    physics_settings->bSubstepping = Config::get<bool>("SIMULATION_CONTROLLER.PHYSICS.ENABLE_SUBSTEPPING");
    physics_settings->MaxSubstepDeltaTime = Config::get<float>("SIMULATION_CONTROLLER.PHYSICS.MAX_SUBSTEP_DELTA_TIME");
    physics_settings->MaxSubsteps = Config::get<int32>("SIMULATION_CONTROLLER.PHYSICS.MAX_SUBSTEPS");

    // Check that the physics substepping parameters match our desired simulation step time.
    // See https://carla.readthedocs.io/en/latest/adv_synchrony_timestep for more details.
    float step_time = Config::get<float>("SIMULATION_CONTROLLER.PHYSICS.SIMULATION_STEP_TIME");
    if (physics_settings->bSubstepping) {
        float max_step_time_allowed_when_substepping = physics_settings->MaxSubstepDeltaTime * physics_settings->MaxSubsteps;
        SP_ASSERT(step_time <= max_step_time_allowed_when_substepping);
    }

    // set fixed simulation step time in seconds, FApp::SetBenchmarking(true) is also needed to enable
    FApp::SetBenchmarking(true);
    FApp::SetFixedDeltaTime(Config::get<double>("SIMULATION_CONTROLLER.PHYSICS.SIMULATION_STEP_TIME"));

    // pause the game
    UGameplayStatics::SetGamePaused(world_, true);

    // create Agent
    agent_ = std::unique_ptr<Agent>(ClassRegistrationUtils::create(Agent::s_class_registrar_, Config::get<std::string>("SIMULATION_CONTROLLER.AGENT"), world_));
    SP_ASSERT(agent_);

    // create Task
    if (Config::get<std::string>("SIMULATION_CONTROLLER.TASK") == "NullTask") {
        task_ = std::make_unique<NullTask>();
    } else if (Config::get<std::string>("SIMULATION_CONTROLLER.TASK") == "ImitationLearningTask") {
        task_ = std::make_unique<ImitationLearningTask>(world_);
    } else {
        SP_ASSERT(false);
    }
    SP_ASSERT(task_);

    // create NavMesh
    nav_mesh_ = std::make_unique<NavMesh>();
    SP_ASSERT(nav_mesh_);

    // create Scene
    scene_ = std::make_unique<Scene>();
    SP_ASSERT(scene_);

    // create Visualizer
    visualizer_ = std::make_unique<Visualizer>(world_);
    SP_ASSERT(visualizer_);

    // deferred initialization for Agent, Task, and Visualizer
    agent_->findObjectReferences(world_);
    task_->findObjectReferences(world_);
    nav_mesh_->findObjectReferences(world_);
    scene_->findObjectReferences(world_);
    visualizer_->findObjectReferences(world_);

    // initialize frame state used for thread synchronization
    frame_state_ = FrameState::Idle;

    // initialize RPC server
    rpc_server_ = std::make_unique<RpcServer>(
        Config::get<std::string>("SIMULATION_CONTROLLER.IP"),
        Config::get<int>("SIMULATION_CONTROLLER.PORT"));
    SP_ASSERT(rpc_server_);

    bindFunctionsToRpcServer();

    int num_worker_threads = 1;
    rpc_server_->runAsync(num_worker_threads);

    has_world_begin_play_executed_ = true;
}

void SimulationController::worldCleanupEventHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    // We only need to perform any additional steps if the world being cleaned up is the world we cached in our world_ member variable.
    if (world == world_) {

        // The worldCleanupEventHandler(...) function is called for all worlds, but some local state (such as rpc_server_ and agent_)
        // is initialized only when worldBeginPlayEventHandler(...) is called for a particular world. So we check if worldBeginPlayEventHandler(...)
        // has been executed.
        if (has_world_begin_play_executed_) {
            has_world_begin_play_executed_ = false;

            SP_ASSERT(rpc_server_);
            rpc_server_->stopRunAsync();
            rpc_server_ = nullptr;

            SP_ASSERT(visualizer_);
            visualizer_->cleanUpObjectReferences();
            visualizer_ = nullptr;

            SP_ASSERT(scene_);
            scene_->cleanUpObjectReferences();
            scene_ = nullptr;

            SP_ASSERT(nav_mesh_);
            nav_mesh_->cleanUpObjectReferences();
            nav_mesh_ = nullptr;

            SP_ASSERT(task_);
            task_->cleanUpObjectReferences();
            task_ = nullptr;

            SP_ASSERT(agent_);
            agent_->cleanUpObjectReferences();
            agent_ = nullptr;
        }

        // remove event handlers bound to this world before world gets cleaned up
        if (Config::get<std::string>("SIMULATION_CONTROLLER.INTERACTION_MODE") == "programmatic") {
            world_->OnWorldBeginPlay.Remove(world_begin_play_handle_);
            world_begin_play_handle_.Reset();
        }

        // clear cached world_ pointer
        world_ = nullptr;
    }
}

void SimulationController::beginFrameEventHandler()
{
    // if begin_tick() has indicated that we should advance the simulation
    if (frame_state_ == FrameState::RequestPreTick) {

        // update frame state, allow begin_tick() to finish executing
        frame_state_ = FrameState::ExecutingPreTick;
        frame_state_executing_pre_tick_promise_.set_value();

        // unpause the game
        UGameplayStatics::SetGamePaused(world_, false);

        // execute all pre-tick synchronous work, wait here for tick() to unblock us
        rpc_server_->runSync();

        // execute task-specific beginFrame() work
        task_->beginFrame();

        // update local state
        frame_state_ = FrameState::ExecutingTick;
    }
}

void SimulationController::endFrameEventHandler()
{
    // if we are currently advancing the simulation
    if (frame_state_ == FrameState::ExecutingTick) {

        // update frame state, allow tick() to finish executing
        frame_state_ = FrameState::ExecutingPostTick;
        frame_state_executing_post_tick_promise_.set_value();

        // execute task-specific endFrame() work
        task_->endFrame();

        // execute all post-tick synchronous work, wait here for endTick() to unblock
        rpc_server_->runSync();

        // pause the game
        UGameplayStatics::SetGamePaused(world_, true);

        // update frame state, allow endTick() to finish executing
        frame_state_ = FrameState::Idle;
        frame_state_idle_promise_.set_value();
    }
}

void SimulationController::bindFunctionsToRpcServer()
{
    rpc_server_->bindAsync("ping", []() -> std::string {
        return "SimulationController received a call to ping()...";
    });

    rpc_server_->bindAsync("get_byte_order", []() -> std::string {
        uint32_t dummy = 0x01020304;
        return (reinterpret_cast<char*>(&dummy)[3] == 1) ? "little" : "big";
    });

    rpc_server_->bindAsync("request_close", []() -> void {
        bool immediate_shutdown = false;
        FGenericPlatformMisc::RequestExit(immediate_shutdown);
    });

    rpc_server_->bindAsync("begin_tick", [this]() -> void {
        SP_ASSERT(frame_state_ == FrameState::Idle);

        // reset promises and futures
        frame_state_idle_promise_ = std::promise<void>();
        frame_state_executing_pre_tick_promise_ = std::promise<void>();
        frame_state_executing_post_tick_promise_ = std::promise<void>();

        frame_state_idle_future_ = frame_state_idle_promise_.get_future();
        frame_state_executing_pre_tick_future_ = frame_state_executing_pre_tick_promise_.get_future();
        frame_state_executing_post_tick_future_ = frame_state_executing_post_tick_promise_.get_future();

        // indicate that we want the game thread to advance the simulation, wait here until frame_state == FrameState::ExecutingPreTick
        frame_state_ = FrameState::RequestPreTick;
        frame_state_executing_pre_tick_future_.wait();

        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
    });

    rpc_server_->bindAsync("tick", [this]() -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);

        // allow beginFrameEventHandler() to finish executing, wait here until frame_state == FrameState::ExecutingPostTick
        rpc_server_->requestStopRunSync();
        frame_state_executing_post_tick_future_.wait();

        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
    });

    rpc_server_->bindAsync("end_tick", [this]() -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);

        // allow endFrameEventHandler() to finish executing, wait here until frame_state == FrameState::Idle
        rpc_server_->requestStopRunSync();
        frame_state_idle_future_.wait();

        SP_ASSERT(frame_state_ == FrameState::Idle);
    });

    rpc_server_->bindAsync("get_action_space", [this]() -> std::map<std::string, ArrayDesc> {
        SP_ASSERT(agent_);
        return agent_->getActionSpace();
    });

    rpc_server_->bindAsync("get_observation_space", [this]() -> std::map<std::string, ArrayDesc> {
        SP_ASSERT(agent_);
        return agent_->getObservationSpace();
    });

    rpc_server_->bindAsync("get_agent_step_info_space", [this]() -> std::map<std::string, ArrayDesc> {
        SP_ASSERT(agent_);
        return agent_->getStepInfoSpace();
    });

    rpc_server_->bindAsync("get_task_step_info_space", [this]() -> std::map<std::string, ArrayDesc> {
        SP_ASSERT(task_);
        return task_->getStepInfoSpace();
    });

    rpc_server_->bindSync("apply_action", [this](std::map<std::string, std::vector<uint8_t>> action) -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        SP_ASSERT(agent_);
        agent_->applyAction(action);
    });

    rpc_server_->bindSync("get_observation", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(agent_);
        return agent_->getObservation();
    });

    rpc_server_->bindSync("get_reward", [this]() -> float {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(task_);
        return task_->getReward();
    });

    rpc_server_->bindSync("is_episode_done", [this]() -> bool {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(task_);
        return task_->isEpisodeDone();
    });

    rpc_server_->bindSync("get_agent_step_info", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(agent_);
        return agent_->getStepInfo();
    });

    rpc_server_->bindSync("get_task_step_info", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(task_);
        return task_->getStepInfo();
    });

    rpc_server_->bindSync("reset_agent", [this]() -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        SP_ASSERT(agent_);
        agent_->reset();
    });

    rpc_server_->bindSync("reset_task", [this]() -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        SP_ASSERT(task_);
        task_->reset();
    });

    rpc_server_->bindSync("is_agent_ready", [this]() -> bool {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(agent_);
        return agent_->isReady();
    });

    rpc_server_->bindSync("is_task_ready", [this]() -> bool {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(task_);
        return task_->isReady();
    });

    rpc_server_->bindSync("navmesh.get_random_points", [this](int num_points) -> std::vector<double> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(nav_mesh_);
        return nav_mesh_->getRandomPoints(num_points);
    });

    rpc_server_->bindSync("navmesh.get_random_reachable_points_in_radius", [this](const std::vector<double>& initial_points, float radius) -> std::vector<double> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(nav_mesh_);
        return nav_mesh_->getRandomReachablePointsInRadius(initial_points, radius);
    });

    rpc_server_->bindSync("navmesh.get_paths", [this](const std::vector<double>& initial_points, const std::vector<double>& goal_points) -> std::vector<std::vector<double>> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(nav_mesh_);
        return nav_mesh_->getPaths(initial_points, goal_points);
    });

    rpc_server_->bindSync("scene.get_all_actor_names", [this]() -> std::vector<std::string> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getAllActorNames();
    });

    rpc_server_->bindSync("scene.get_all_scene_component_names", [this]() -> std::vector<std::string> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getAllSceneComponentNames();
    });
    
    rpc_server_->bindSync("scene.get_all_actor_locations", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getAllActorLocations();
    });

    rpc_server_->bindSync("scene.get_all_actor_rotations", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getAllActorRotations();
    });

    rpc_server_->bindSync("scene.get_all_component_world_locations", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getAllComponentWorldLocations();
    });

    rpc_server_->bindSync("scene.get_all_component_world_rotations", [this]() -> std::map<std::string, std::vector<uint8_t>> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getAllComponentWorldRotations();
    });

    rpc_server_->bindSync("scene.get_actor_locations", [this](std::vector<std::string> actor_names) -> std::vector<std::uint8_t> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getActorLocations(actor_names);
    });

    rpc_server_->bindSync("scene.get_actor_rotations", [this](std::vector<std::string> actor_names) -> std::vector<std::uint8_t> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getActorRotations(actor_names);
    });

    rpc_server_->bindSync("scene.get_component_world_locations", [this](std::vector<std::string> component_names) -> std::vector<std::uint8_t> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getComponentWorldLocations(component_names);
    });

    rpc_server_->bindSync("scene.get_component_world_rotations", [this](std::vector<std::string> component_names) -> std::vector<std::uint8_t> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getComponentWorldRotations(component_names);
    });

    rpc_server_->bindSync("scene.get_static_mesh_components_for_actors", [this](std::vector<std::string> actor_names) -> std::map<std::string, std::vector<std::string>> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getStaticMeshComponentsForActors(actor_names);
    });

    rpc_server_->bindSync("scene.get_physics_constraint_components_for_actors", [this](std::vector<std::string> actor_names) -> std::map<std::string, std::vector<std::string>> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->getPhysicsConstraintComponentsForActors(actor_names);
    });

    rpc_server_->bindSync("scene.is_using_absolute_location", [this](std::vector<std::string> object_names) -> std::vector<bool> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->isUsingAbsoluteLocation(object_names);
    });

    rpc_server_->bindSync("scene.is_using_absolute_rotation", [this](std::vector<std::string> object_names) -> std::vector<bool> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->isUsingAbsoluteRotation(object_names);
    });

    rpc_server_->bindSync("scene.is_using_absolute_scale", [this](std::vector<std::string> object_names) -> std::vector<bool> {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPostTick);
        SP_ASSERT(scene_);
        return scene_->isUsingAbsoluteScale(object_names);
    });

    rpc_server_->bindSync("scene.set_absolute", [this](std::vector<std::string> object_names, std::vector<bool> blocations, std::vector<bool> brotations, std::vector<bool> bscales) -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        SP_ASSERT(scene_);
        scene_->SetAbolute(object_names, blocations, brotations, bscales);
    });

    rpc_server_->bindSync("scene.set_actor_locations", [this](std::map<std::string, std::vector<uint8_t>> actor_locations) -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        SP_ASSERT(scene_);
        scene_->setActorLocations(actor_locations);
    });

    rpc_server_->bindSync("scene.set_actor_rotations", [this](std::map<std::string, std::vector<uint8_t>> actor_rotations) -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        SP_ASSERT(scene_);
        scene_->setActorRotations(actor_rotations);
    });

    rpc_server_->bindSync("scene.set_component_world_locations", [this](std::map<std::string, std::vector<uint8_t>> component_locations) -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        SP_ASSERT(scene_);
        scene_->setComponentWorldLocations(component_locations);
    });
    
    rpc_server_->bindSync("scene.set_component_world_rotations", [this](std::map<std::string, std::vector<uint8_t>> component_rotations) -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        SP_ASSERT(scene_);
        scene_->setComponentWorldRotations(component_rotations);
    });

    rpc_server_->bindSync("scene.set_component_relative_locations", [this](std::map<std::string, std::vector<uint8_t>> component_locations) -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        SP_ASSERT(scene_);
        scene_->setComponentRelativeLocations(component_locations);
    });

    rpc_server_->bindSync("scene.set_component_relative_rotations", [this](std::map<std::string, std::vector<uint8_t>> component_rotations) -> void {
        SP_ASSERT(frame_state_ == FrameState::ExecutingPreTick);
        SP_ASSERT(scene_);
        scene_->setComponentRelativeRotations(component_rotations);
    });
}

IMPLEMENT_MODULE(SimulationController, SimulationController)
