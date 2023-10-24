//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpearSimEditor/DebugWidget.h"

#include <Containers/UnrealString.h> // FString
#include <Engine/World.h>
#include <GameFramework/Actor.h>
#include <Math/Rotator.h>
#include <Math/Vector.h>
#include <Misc/Paths.h>

#include "CoreUtils/Log.h"
#include "CoreUtils/Std.h"
#include "CoreUtils/Unreal.h"
#include "UrdfRobot/UrdfRobotPawn.h"
#include "Vehicle/VehiclePawn.h"

ADebugWidget::ADebugWidget()
{
    SP_LOG_CURRENT_FUNCTION();
}

ADebugWidget::~ADebugWidget()
{
    SP_LOG_CURRENT_FUNCTION();

    DebugString = Unreal::toFString("");
    UrdfFile = Unreal::toFString("");
}

void ADebugWidget::LoadConfig()
{
    AActor::LoadConfig();
}

void ADebugWidget::SaveConfig()
{
    AActor::SaveConfig();
}

void ADebugWidget::PrintDebugString()
{
    SP_LOG("DebugString: ", Unreal::toStdString(DebugString));
}

void ADebugWidget::SpawnVehiclePawn()
{
    UWorld* world = GetWorld();
    SP_ASSERT(world);

    static int i = 0;
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    FActorSpawnParameters actor_spawn_parameters;
    actor_spawn_parameters.Name = Unreal::toFName("vehicle_pawn_" + Std::toString(i++));
    actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AVehiclePawn* vehicle_pawn = world->SpawnActor<AVehiclePawn>(spawn_location, spawn_rotation, actor_spawn_parameters);
    SP_ASSERT(vehicle_pawn);
}

void ADebugWidget::SpawnUrdfRobotPawn()
{
    UWorld* world = GetWorld();
    SP_ASSERT(world);

    static int i = 1;
    FVector spawn_location = FVector::ZeroVector;
    FRotator spawn_rotation = FRotator::ZeroRotator;
    FActorSpawnParameters actor_spawn_parameters;
    actor_spawn_parameters.Name = Unreal::toFName("urdf_robot_pawn_" + Std::toString(i++));
    actor_spawn_parameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
    AUrdfRobotPawn* urdf_robot_pawn = world->SpawnActor<AUrdfRobotPawn>(spawn_location, spawn_rotation, actor_spawn_parameters);
    SP_ASSERT(urdf_robot_pawn);

    urdf_robot_pawn->UrdfFile = UrdfFile;
    urdf_robot_pawn->Initialize();
}
