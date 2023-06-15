//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/VehicleAgent.h"

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Components/BoxComponent.h>
#include <Components/PrimitiveComponent.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>

#include "CoreUtils/ArrayDesc.h"
#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "SimulationController/CameraSensor.h"
#include "SimulationController/ImuSensor.h"
//#include "Vehicle/VehicleMovementComponent.h"
#include "Vehicle/VehiclePawn.h"

VehicleAgent::VehicleAgent(UWorld* world)
{
    SP_LOG_CURRENT_FUNCTION();

    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    std::string spawn_mode = Config::get<std::string>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_MODE");
    if (spawn_mode == "specify_existing_actor") {
        AActor* spawn_actor = Unreal::findActorByName(world, Config::get<std::string>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_ACTOR_NAME"));
        SP_ASSERT(spawn_actor);
        spawn_location = spawn_actor->GetActorLocation();
        spawn_rotation = spawn_actor->GetActorRotation();
    }
    else if (spawn_mode == "specify_pose") {
        spawn_location = FVector(
            Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_LOCATION_X"),
            Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_LOCATION_Y"),
            Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_LOCATION_Z"));
        spawn_rotation = FRotator(
            Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_PITCH"),
            Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_YAW"),
            Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.SPAWN_ROLL"));
    }
    else {
        SP_ASSERT(false);
    }
    FActorSpawnParameters actor_spawn_params;
    actor_spawn_params.Name = Unreal::toFName(Config::get<std::string>("SIMULATION_CONTROLLER.VEHICLE_AGENT.VEHICLE_ACTOR_NAME"));
    actor_spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    vehicle_pawn_ = world->SpawnActor<AVehiclePawn>(spawn_location, spawn_rotation, actor_spawn_params);
    SP_ASSERT(vehicle_pawn_);

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "camera")) {
        camera_sensor_ = std::make_unique<CameraSensor>(
            vehicle_pawn_->camera_component_,
            Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.RENDER_PASSES"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.IMAGE_WIDTH"),
            Config::get<unsigned int>("SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.IMAGE_HEIGHT"),
            Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.CAMERA.FOV"));
        SP_ASSERT(camera_sensor_);
    }

    if (Std::contains(observation_components, "imu")) {
        imu_sensor_ = std::make_unique<ImuSensor>(vehicle_pawn_->imu_component_);
        SP_ASSERT(imu_sensor_);
    }
}

VehicleAgent::~VehicleAgent()
{
    SP_LOG_CURRENT_FUNCTION();

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "imu")) {
        SP_ASSERT(imu_sensor_);
        imu_sensor_ = nullptr;
    }

    if (Std::contains(observation_components, "camera")) {
        SP_ASSERT(camera_sensor_);
        camera_sensor_ = nullptr;
    }

    SP_ASSERT(vehicle_pawn_);
    vehicle_pawn_->Destroy();
    vehicle_pawn_ = nullptr;
}

void VehicleAgent::findObjectReferences(UWorld* world) {}

void VehicleAgent::cleanUpObjectReferences() {}

std::map<std::string, ArrayDesc> VehicleAgent::getActionSpace() const
{
    std::map<std::string, ArrayDesc> action_space;
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "set_drive_torques")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = { 4 };
        array_desc.datatype_ = DataType::Float64;
        action_space["set_drive_torques"] = std::move(array_desc);
    }

    if (Std::contains(action_components, "set_brake_torques")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.shape_ = { 4 };
        array_desc.datatype_ = DataType::Float64;
        action_space["set_brake_torques"] = std::move(array_desc);
    }

    return action_space;
}

std::map<std::string, ArrayDesc> VehicleAgent::getObservationSpace() const
{
    std::map<std::string, ArrayDesc> observation_space;
    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "location")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { 3 };
        observation_space["location"] = std::move(array_desc); // position (X, Y, Z) in [cms] of the agent relative to the world frame.
    }

    if (Std::contains(observation_components, "rotation")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { 3 };
        observation_space["rotation"] = std::move(array_desc); // orientation (Roll, Pitch, Yaw) in [degs] of the agent relative to the world frame.
    }

    if (Std::contains(observation_components, "wheel_encoder")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { 4 };
        observation_space["wheel_encoder"] = std::move(array_desc); // FL, FR, RL, RR, in [rad/s]
    }

    if (Std::contains(observation_components, "imu")) {
        ArrayDesc array_desc;
        array_desc.low_ = std::numeric_limits<double>::lowest();
        array_desc.high_ = std::numeric_limits<double>::max();
        array_desc.datatype_ = DataType::Float64;
        array_desc.shape_ = { 6 };
        observation_space["imu"] = std::move(array_desc); // a_x, a_y, a_z in [cm/s^s] g_x, g_y, g_z in [rad/s]
    }

    std::map<std::string, ArrayDesc> camera_sensor_observation_space = camera_sensor_->getObservationSpace(observation_components);
    for (auto& camera_sensor_observation_space_component : camera_sensor_observation_space) {
        observation_space[camera_sensor_observation_space_component.first] = std::move(camera_sensor_observation_space_component.second);
    }

    return observation_space;
}

std::map<std::string, ArrayDesc> VehicleAgent::getStepInfoSpace() const
{
    return {};
}

void VehicleAgent::applyAction(const std::map<std::string, std::vector<uint8_t>>& action)
{
    auto action_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.ACTION_COMPONENTS");

    if (Std::contains(action_components, "set_drive_torques")) {
        vehicle_pawn_->setDriveTorques(Std::reinterpretAs<double>(action.at("set_drive_torques")));
        vehicle_pawn_->setBrakeTorques(std::vector<double>{0.0, 0.0, 0.0, 0.0});
    } else if (Std::contains(action_components, "set_brake_torques")) {
        vehicle_pawn_->setBrakeTorques(Std::reinterpretAs<double>(action.at("set_brake_torques")));
    }
}

std::map<std::string, std::vector<uint8_t>> VehicleAgent::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    auto observation_components = Config::get<std::vector<std::string>>("SIMULATION_CONTROLLER.VEHICLE_AGENT.OBSERVATION_COMPONENTS");

    if (Std::contains(observation_components, "location")) {
        FVector location = vehicle_pawn_->GetActorLocation();
        observation["location"] = Std::reinterpretAs<uint8_t>(std::vector<double>{
            location.X,
            location.Y,
            location.Z});
    }

    if (Std::contains(observation_components, "rotation")) {
        FRotator rotation = vehicle_pawn_->GetActorRotation();
        observation["rotation"] = Std::reinterpretAs<uint8_t>(std::vector<double>{
            rotation.Pitch,
            rotation.Yaw,
            rotation.Roll});
    }

    if (Std::contains(observation_components, "wheel_encoder")) {
        //observation["wheel_encoder"] = Std::reinterpretAs<uint8_t>(vehicle_pawn_->vehicle_movement_component_->getWheelRotationSpeeds());
        observation["wheel_encoder"] = Std::reinterpretAs<uint8_t>(vehicle_pawn_->getWheelRotationSpeeds());
    }

    if (Std::contains(observation_components, "imu")) {
        observation["imu"] = Std::reinterpretAs<uint8_t>(std::vector<double>{
            imu_sensor_->linear_acceleration_body_.X,
            imu_sensor_->linear_acceleration_body_.Y,
            imu_sensor_->linear_acceleration_body_.Z,
            imu_sensor_->angular_velocity_body_.X,
            imu_sensor_->angular_velocity_body_.Y,
            imu_sensor_->angular_velocity_body_.Z});
    }

    std::map<std::string, std::vector<uint8_t>> camera_sensor_observation = camera_sensor_->getObservation(observation_components);
    for (auto& camera_sensor_observation_component : camera_sensor_observation) {
        observation[camera_sensor_observation_component.first] = std::move(camera_sensor_observation_component.second);
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> VehicleAgent::getStepInfo() const
{
    return {};
}

void VehicleAgent::reset()
{
    // For some reason, the pose of AVehiclePawn needs to be set using ETeleportType::TeleportPhysics, which maintains
    // velocity information across calls to SetActorPositionAndRotation(...). Since tasks are supposed to be implemented
    // in a general way, they must therefore use ETeleportType::TeleportPhysics to set the pose of actors, because the
    // actor they're attempting to reset might be an AVehiclePawn. But this means that our velocity will be maintained
    // unless we explicitly reset it, so we reset our velocity here.
    vehicle_pawn_->skeletal_mesh_component_->SetPhysicsLinearVelocity(FVector::ZeroVector, false);
    vehicle_pawn_->skeletal_mesh_component_->SetPhysicsAngularVelocityInRadians(FVector::ZeroVector, false);
    vehicle_pawn_->skeletal_mesh_component_->GetBodyInstance()->ClearTorques();
    vehicle_pawn_->skeletal_mesh_component_->GetBodyInstance()->ClearForces();

    // Reset vehicle
    //vehicle_pawn_->vehicle_movement_component_->ResetVehicle();
    vehicle_pawn_->resetVehicle();
    vehicle_pawn_->setBrakeTorques(std::vector<double>{1000.0f, 1000.0f, 1000.0f, 1000.0f}); // TODO: get value from the config system
}

bool VehicleAgent::isReady() const
{
    return vehicle_pawn_->GetVelocity().Size() <= Config::get<float>("SIMULATION_CONTROLLER.VEHICLE_AGENT.IS_READY_VELOCITY_THRESHOLD");
}
