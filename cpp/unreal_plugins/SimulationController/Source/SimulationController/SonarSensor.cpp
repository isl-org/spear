// TODO: uncomment when we're ready to re-enable Sonar sensor
/*
//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/SonarSensor.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include <Components/BoxComponent.h>
#include <Delegates/IDelegateInstance.h>
#include <DrawDebugHelpers.h>
#include <Engine/EngineBaseTypes.h>
#include <Engine/HitResult.h>
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Math/UnrealMathUtility.h>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Log.h"
#include "SimulationController/TickEventComponent.h"

SonarSensor::SonarSensor(UBoxComponent* component)
{
    SP_LOG_CURRENT_FUNCTION();

    ASSERT(component);
    box_component_ = component;

    parent_actor_ = box_component_->GetWorld()->SpawnActor<AActor>();
    ASSERT(parent_actor_);

    tick_event_component_ = NewObject<UTickEventComponent>(parent_actor_);
    ASSERT(tick_event_component_);
    tick_event_component_->RegisterComponent();
    tick_event_component_->PrimaryComponentTick.TickGroup = ETickingGroup::TG_PostPhysics;
    tick_event_component_handle_ = tick_event_component_->delegate_.AddRaw(this, &SonarSensor::postPhysicsPreRenderTickEventHandler);

    range_ = Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX");
}

SonarSensor::~SonarSensor()
{
    SP_LOG_CURRENT_FUNCTION();

    ASSERT(tick_event_component_);
    tick_event_component_->delegate_.Remove(tick_event_component_handle_);
    tick_event_component_handle_.Reset();
    tick_event_component_->DestroyComponent();
    tick_event_component_ = nullptr;

    ASSERT(parent_actor_);
    parent_actor_->Destroy();
    parent_actor_ = nullptr;
}

void SonarSensor::postPhysicsPreRenderTickEventHandler(float delta_time, ELevelTick level_tick)
{
    float world_to_meters = box_component_->GetWorld()->GetWorldSettings()->WorldToMeters;

    FCollisionQueryParams collision_query_params;
    collision_query_params.bTraceComplex = true;
    collision_query_params.bReturnPhysicalMaterial = false;

    // Maximum sonar radius in horizontal and vertical direction
    float max_rx =
        std::tanf(FMath::DegreesToRadians(Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.HORIZONTAL_FOV") * 0.5f)) *
        Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX") *
        world_to_meters;
    float max_ry =
        std::tanf(FMath::DegreesToRadians(Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.VERTICAL_FOV") * 0.5f)) *
        Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX") *
        world_to_meters;
    FTransform sensor_transform = box_component_->GetComponentTransform();
    FRotator transform_rotator  = sensor_transform.Rotator();
    FVector start_location      = sensor_transform.GetLocation();
    FVector transform_x_axis    = transform_rotator.RotateVector(sensor_transform.GetUnitAxis(EAxis::X));
    FVector transform_y_axis    = transform_rotator.RotateVector(sensor_transform.GetUnitAxis(EAxis::Y));
    FVector transform_z_axis    = transform_rotator.RotateVector(sensor_transform.GetUnitAxis(EAxis::Z));

    std::vector<bool> hits;
    hits.clear();
    hits.resize(Config::get<unsigned int>("SIMULATION_CONTROLLER.SONAR_SENSOR.NUM_RAYS"));

    std::vector<FHitResult> hit_results;
    hit_results.clear();
    hit_results.resize(Config::get<unsigned int>("SIMULATION_CONTROLLER.SONAR_SENSOR.NUM_RAYS"));

    float min_distance = Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX");

    for (int i = 0; i < Config::get<int>("SIMULATION_CONTROLLER.SONAR_SENSOR.NUM_RAYS"); i++) {
        hit_results[i] = FHitResult(EForceInit::ForceInit);
        float distance = Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX");
        float radius = std::uniform_real_distribution<float>()(random_gen_);
        float angle = std::uniform_real_distribution<float>(0.0f, 2.0f * PI)(random_gen_); // Uniform distibution of vales between 0 and 2*PI
        FVector end_location = start_location + transform_rotator.RotateVector({
                Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX") * world_to_meters,
                max_rx * radius * std::cosf(angle),
                max_ry * radius * std::sinf(angle) });

        bool hit = box_component_->GetWorld()->LineTraceSingleByChannel(
            hit_results.at(i),
            start_location,
            end_location,
            ECollisionChannel::ECC_Visibility, collision_query_params, FCollisionResponseParams::DefaultResponseParam);

        // If we hit anything...
        if (hit && hit_results.at(i).HitObjectHandle.IsValid()) {
            FVector ray = (hit_results.at(i).ImpactPoint - start_location) / world_to_meters;

            // If the angle of hit surface normal and sonar ray is greater than MAX_REFLECTION_ANGLE, then no reflection should be observed
            if (abs(FVector::DotProduct(hit_results.at(i).Normal, ray / ray.Size())) <
                std::cosf(FMath::DegreesToRadians(Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.MAX_REFLECTION_ANGLE")))) {
                hits[i] = false;
                distance = Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX");

            } else { // A proper distance measurement can be made
                hits[i] = true;
                distance = std::max(Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MIN"), hit_results.at(i).Distance / world_to_meters);
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }

        } else { // If we didn't hit anything...
            hits[i] = false;
            distance = Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX");
        }
    }

    range_ = min_distance + Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.NOISE_STD_DEV") * std::uniform_real_distribution<float>()(random_gen_);

    // Draw the sensing cone and the sonar rays
    if (Config::get<bool>("SIMULATION_CONTROLLER.SONAR_SENSOR.DEBUG_RENDER")) {

        FVector sensing_cone_vertex_1 = start_location + transform_rotator.RotateVector({
            Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX") * world_to_meters,
            max_rx,
            max_ry });

        FVector sensing_cone_vertex_2 = start_location + transform_rotator.RotateVector({
            Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX") * world_to_meters,
            -max_rx,
            max_ry });

        FVector sensing_cone_vertex_3 = start_location + transform_rotator.RotateVector({
            Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX") * world_to_meters,
            -max_rx,
            -max_ry });

        FVector sensing_cone_vertex_4 = start_location + transform_rotator.RotateVector({
            Config::get<float>("SIMULATION_CONTROLLER.SONAR_SENSOR.RANGE.MAX") * world_to_meters,
            max_rx,
            -max_ry });

        DrawDebugDirectionalArrow(box_component_->GetWorld(), start_location, sensing_cone_vertex_1, 0.15f, FColor(255, 0, 0), false, 0.033f, 0, 0.15f);
        DrawDebugDirectionalArrow(box_component_->GetWorld(), start_location, sensing_cone_vertex_2, 0.15f, FColor(255, 0, 0), false, 0.033f, 0, 0.15f);
        DrawDebugDirectionalArrow(box_component_->GetWorld(), start_location, sensing_cone_vertex_3, 0.15f, FColor(255, 0, 0), false, 0.033f, 0, 0.15f);
        DrawDebugDirectionalArrow(box_component_->GetWorld(), start_location, sensing_cone_vertex_4, 0.15f, FColor(255, 0, 0), false, 0.033f, 0, 0.15f);
        DrawDebugLine(box_component_->GetWorld(), sensing_cone_vertex_1, sensing_cone_vertex_2, FColor(255, 0, 0), false, 0.033f, 0, 0.15f);
        DrawDebugLine(box_component_->GetWorld(), sensing_cone_vertex_2, sensing_cone_vertex_3, FColor(255, 0, 0), false, 0.033f, 0, 0.15f);
        DrawDebugLine(box_component_->GetWorld(), sensing_cone_vertex_3, sensing_cone_vertex_4, FColor(255, 0, 0), false, 0.033f, 0, 0.15f);
        DrawDebugLine(box_component_->GetWorld(), sensing_cone_vertex_4, sensing_cone_vertex_1, FColor(255, 0, 0), false, 0.033f, 0, 0.15f);

        for (int i = 0; i < Config::get<int>("SIMULATION_CONTROLLER.SONAR_SENSOR.NUM_RAYS"); i++) {

            DrawDebugLine(box_component_->GetWorld(), start_location, hit_results.at(i).ImpactPoint, FColor(200, 0, 200), false, 0.033f, 0, 0.15f);
            DrawDebugDirectionalArrow(
                box_component_->GetWorld(),
                hit_results.at(i).ImpactPoint,
                hit_results.at(i).ImpactPoint + 5.0f * hit_results.at(i).Normal,
                0.15f, FColor(0, 188, 227), false, 0.033f, 0, 0.15f);

            if (hits.at(i)) {
                DrawDebugPoint(box_component_->GetWorld(), hit_results.at(i).ImpactPoint, 5.0f, FColor(0, 255, 0), false, 0.033f, 0);
            } else {
                DrawDebugPoint(box_component_->GetWorld(), hit_results.at(i).ImpactPoint, 5.0f, FColor(0, 0, 255), false, 0.033f, 0);
            }
        }
    }   
}
*/
