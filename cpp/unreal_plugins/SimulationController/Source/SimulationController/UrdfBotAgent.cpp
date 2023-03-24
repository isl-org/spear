//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/UrdfBotAgent.h"

#include <Components/SceneCaptureComponent2D.h>
#include <GameFramework/PlayerStart.h>
#include <Kismet/GameplayStatics.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Box.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/CameraSensor.h"
#include "SimulationController/ImuSensor.h"
#include "SimulationController/SonarSensor.h"
#include "UrdfBot/UrdfBotPawn.h"
#include "UrdfBot/UrdfRobotComponent.h"

UrdfBotAgent::UrdfBotAgent(UWorld* world)
{
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    std::string spawn_mode = Config::get<std::string>("SIMULATION_CONTROLLER.URDFBOT_AGENT.SPAWN_MODE");
    if (spawn_mode == "player_start") {
        AActor* player_start = UGameplayStatics::GetActorOfClass(world, APlayerStart::StaticClass());
        ASSERT(player_start);
        spawn_location = player_start->GetActorLocation();
        spawn_rotation = player_start->GetActorRotation();
    } else if (spawn_mode == "world_transform") {
        spawn_location = FVector(
            Config::get<float>("URDFBOT.URDFBOT_PAWN.POSITION_X"),
            Config::get<float>("URDFBOT.URDFBOT_PAWN.POSITION_Y"),
            Config::get<float>("URDFBOT.URDFBOT_PAWN.POSITION_Z"));
    
        spawn_rotation = FRotator(
            Config::get<float>("URDFBOT.URDFBOT_PAWN.PITCH"),
            Config::get<float>("URDFBOT.URDFBOT_PAWN.YAW"),
            Config::get<float>("URDFBOT.URDFBOT_PAWN.ROLL"));
    } else {
        ASSERT(false);
    }

    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.URDFBOT_AGENT.URDFBOT_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    urdf_bot_pawn_ = world->SpawnActor<AUrdfBotPawn>(spawn_location, spawn_rotation, actor_spawn_params);
    ASSERT(urdf_bot_pawn_);

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            urdf_bot_pawn_->camera_component_,
            Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.URDFBOT_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.URDFBOT_AGENT.CAMERA.IMAGE_HEIGHT"));
        ASSERT(camera_sensor_);

        // update FOV
        for (auto& pass : Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.CAMERA.RENDER_PASSES")) {
            camera_sensor_->render_passes_.at(pass).scene_capture_component_->FOVAngle =
                Config::get<float>("SIMULATION_CONTROLLER.URDFBOT_AGENT.CAMERA.FOV");
        }
    }
}

UrdfBotAgent::~UrdfBotAgent()
{
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;
    }

    ASSERT(urdf_bot_pawn_);
    urdf_bot_pawn_->Destroy();
    urdf_bot_pawn_ = nullptr;
}

void UrdfBotAgent::findObjectReferences(UWorld* world) {}

void UrdfBotAgent::cleanUpObjectReferences() {}

std::map<std::string, Box> UrdfBotAgent::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.ACTION_COMPONENTS");

    std::map<std::string, Box> robot_component_action_space = urdf_bot_pawn_->urdf_robot_component_->getActionSpace(action_components);
    for (auto& robot_component_observation_action_component : robot_component_action_space) {
        action_space[robot_component_observation_action_component.first] = std::move(robot_component_observation_action_component.second);
    }

    return action_space;
}

std::map<std::string, Box> UrdfBotAgent::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.OBSERVATION_COMPONENTS");
    
    std::map<std::string, Box> robot_component_observation_space = urdf_bot_pawn_->urdf_robot_component_->getObservationSpace(observation_components);
    for (auto& robot_component_observation_space_component : robot_component_observation_space) {
        observation_space[robot_component_observation_space_component.first] = std::move(robot_component_observation_space_component.second);
    }

    std::map<std::string, Box> camera_sensor_observation_space = camera_sensor_->getObservationSpace(observation_components);
    for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
        observation_space[camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
    }

    return observation_space;
}

std::map<std::string, Box> UrdfBotAgent::getStepInfoSpace() const
{
    return {};
}

void UrdfBotAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& actions)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "control_joints")) {
        urdf_bot_pawn_->urdf_robot_component_->applyAction(actions);
    }
}

std::map<std::string, std::vector<uint8_t>> UrdfBotAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.URDFBOT_AGENT.OBSERVATION_COMPONENTS");

    std::map<std::string, std::vector<uint8_t>> robot_component_observation = urdf_bot_pawn_->urdf_robot_component_->getObservation(observation_components);
    for (auto& robot_component_observation_component : robot_component_observation) {
        observation[robot_component_observation_component.first] = std::move(robot_component_observation_component.second);
    }

    std::map<std::string, std::vector<uint8_t>> camera_sensor_observation = camera_sensor_->getObservation(observation_components);
    for (auto& camera_sensor_observation_component : camera_sensor_observation) {
        observation[camera_sensor_observation_component.first] = std::move(camera_sensor_observation_component.second);
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> UrdfBotAgent::getStepInfo() const
{
    return {};
}

void UrdfBotAgent::reset() {}

bool UrdfBotAgent::isReady() const
{
    return urdf_bot_pawn_->urdf_robot_component_->GetComponentVelocity().Size() <=
           Config::get<float>("SIMULATION_CONTROLLER.URDFBOT_AGENT.IS_READY_VELOCITY_THRESHOLD");
}
