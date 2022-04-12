#pragma once

#include <Engine/World.h>
#include <GameFramework/Actor.h>

class Visualizer
{
public:
    Visualizer(UWorld* world)
    {
        for (TActorIterator<AActor> actor_itr(world, AActor::StaticClass()); actor_itr; ++actor_itr) {
            std::string actor_name = TCHAR_TO_UTF8(*(*actor_itr)->GetName());
            if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_ACTOR_NAME"})) {
                ASSERT(!visualizer_camera_);
                visualizer_camera_ = *actor_itr;
                break;
            }
        }
        ASSERT(visualizer_camera_);

        // set active camera
        APlayerController* Controller = world->GetFirstPlayerController();
        ASSERT(Controller);
        Controller->SetViewTarget(visualizer_camera_);

        // setup camera pose
        if (Config::getValue<bool>({"SIMULATION_CONTROLLER", "VISUALIZER", "SET_CAMERA_POSE"})) {
            const FVector camera_pose(
                Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_POSITION_X"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_POSITION_Y"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_POSITION_Z"}));
            visualizer_camera_->SetActorLocation(camera_pose);
            visualizer_camera_->SetActorRotation(FRotator(
                Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_PITCH"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_YAW"}),
                Config::getValue<float>({"SIMULATION_CONTROLLER", "VISUALIZER", "CAMERA_ROLL"})));
        }
    }

    ~Visualizer()
    {
        ASSERT(visualizer_camera_);
        visualizer_camera_ = nullptr;
    }
private:
    AActor* visualizer_camera_ = nullptr;
};
