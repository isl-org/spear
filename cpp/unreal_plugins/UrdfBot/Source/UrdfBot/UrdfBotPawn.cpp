//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBotPawn.h"

#include <iostream>

#include <Camera/CameraComponent.h>
#include <Components/InputComponent.h>

#include "CoreUtils/Config.h"
#include "UrdfRobotComponent.h"

AUrdfBotPawn::AUrdfBotPawn(const FObjectInitializer& object_initializer): APawn(object_initializer)
{
    // setup robot
    robot_component_ = CreateDefaultSubobject<UUrdfRobotComponent>(TEXT("RobotComponent"));
    RootComponent = Cast<USceneComponent>(robot_component_->root_link_component_);

    // setup camera
    FVector camera_location(
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_X"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_Y"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.POSITION_Z"));

    FRotator camera_orientation(
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.PITCH"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.YAW"),
        Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.ROLL"));

    camera_component_ = CreateDefaultSubobject<UCameraComponent>(TEXT("AOpenBotPawn::camera_component_"));
    ASSERT(camera_component_);

    camera_component_->SetRelativeLocationAndRotation(camera_location, camera_orientation);
    camera_component_->SetupAttachment(RootComponent);
    camera_component_->bUsePawnControlRotation = false;
    camera_component_->FieldOfView = Config::get<float>("URDFBOT.URDFBOT_PAWN.CAMERA_COMPONENT.FOV");
}

void AUrdfBotPawn::SetupPlayerInputComponent(class UInputComponent* input_component)
{
    Super::SetupPlayerInputComponent(input_component);
}

void AUrdfBotPawn::Tick(float delta_time)
{
    Super::Tick(delta_time);
}
