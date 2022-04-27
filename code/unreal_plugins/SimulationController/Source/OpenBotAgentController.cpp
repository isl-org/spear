#include "OpenBotAgentController.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Components/SceneCaptureComponent2D.h>
#include <Components/StaticMeshComponent.h>
#include <Engine/TextureRenderTarget2D.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <PxRigidDynamic.h>
#include <SimpleWheeledVehicleMovementComponent.h>
#include <UObject/UObjectGlobals.h>

#include <PIPCamera.h>
#include <SimpleVehicle/SimpleVehiclePawn.h>

#include "Assert.h"
#include "Box.h"
#include "Config.h"
#include "Serialize.h"

OpenBotAgentController::OpenBotAgentController(UWorld* world)
{
    for (TActorIterator<AActor> actor_itr(world, AActor::StaticClass()); actor_itr; ++actor_itr) {
        std::string actor_name = TCHAR_TO_UTF8(*(*actor_itr)->GetName());
        if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "AGENT_ACTOR_NAME"})) {
            ASSERT(!agent_actor_);
            agent_actor_ = *actor_itr;
        } else if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "GOAL_ACTOR_NAME"})) {
            ASSERT(!goal_actor_);
            goal_actor_ = *actor_itr;
        }
    }
    ASSERT(agent_actor_);
    ASSERT(goal_actor_);

    // setup observation camera
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {

        TArray<AActor*> all_attached_actors;
        agent_actor_->GetAttachedActors(all_attached_actors, true);

        for (const auto& actor : all_attached_actors) {
            std::string actor_name = TCHAR_TO_UTF8(*actor->GetName());
            if (actor_name == Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "OBSERVATION_CAMERA_ACTOR_NAME"})) {
                ASSERT(!observation_camera_actor_);
                observation_camera_actor_ = actor;
                break;
            }
        }
        ASSERT(observation_camera_actor_);
        
        // create SceneCaptureComponent2D and TextureRenderTarget2D
        APIPCamera* pip_camera = dynamic_cast<APIPCamera*>(observation_camera_actor_);
        ASSERT(pip_camera);
        scene_capture_component_ = pip_camera->GetSceneCaptureComponent();
        ASSERT(scene_capture_component_);

        // Set Camera Properties
        scene_capture_component_->bAlwaysPersistRenderingState = 1;
        scene_capture_component_->bCaptureEveryFrame = 0;
        scene_capture_component_->FOVAngle = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "SMARTPHONE_FOV"}); // Smartphone FOV
        scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        scene_capture_component_->ShowFlags.SetTemporalAA(false);
        scene_capture_component_->ShowFlags.SetAntiAliasing(true);

        new_object_parent_actor_ = world->SpawnActor<AActor>();
        ASSERT(new_object_parent_actor_);

        // Adjust RenderTarget
        texture_render_target_ = NewObject<UTextureRenderTarget2D>(new_object_parent_actor_, TEXT("TextureRenderTarget2D"));
        ASSERT(texture_render_target_);

        texture_render_target_->TargetGamma = GEngine->GetDisplayGamma(); // Set FrameWidth and FrameHeight: 1.2f; for Vulkan | GEngine->GetDisplayGamma(); for DX11/12
        texture_render_target_->InitAutoFormat(Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}),
                                               Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"})); // Setup the RenderTarget capture format: some random format, got crashing otherwise frameWidht = 2048 and frameHeight = 2048.
        texture_render_target_->InitCustomFormat(Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}),
                                                 Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"}),
                                                 PF_B8G8R8A8,
                                                 true); // PF_B8G8R8A8 disables HDR which will boost storing to disk due to less image information
        texture_render_target_->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
        texture_render_target_->bGPUSharedFlag = true; // demand buffer on GPU
        scene_capture_component_->TextureTarget = texture_render_target_;

        // Set post processing parameters:
        FPostProcessSettings post_process_settings;
        post_process_settings.MotionBlurAmount = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "MOTION_BLUR_AMOUNT"}); // Strength of motion blur, 0:off, should be renamed to intensity
        post_process_settings.MotionBlurMax = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "MOTION_BLUR_MAX"});    // Max distortion caused by motion blur, in percent of the screen width, 0:off
        scene_capture_component_->PostProcessSettings = post_process_settings;
        scene_capture_component_->PostProcessBlendWeight = Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "POST_PROC_BLEND_WEIGHT"}); // Range (0.0, 1.0) where 0 indicates no effect, 1 indicates full effect.
    }
}

OpenBotAgentController::~OpenBotAgentController()
{
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        
        ASSERT(texture_render_target_);
        texture_render_target_->MarkPendingKill();
        texture_render_target_ = nullptr;

        ASSERT(new_object_parent_actor_);
        new_object_parent_actor_->Destroy();
        new_object_parent_actor_ = nullptr;

        ASSERT(scene_capture_component_);
        scene_capture_component_ = nullptr;

        ASSERT(observation_camera_actor_);
        observation_camera_actor_ = nullptr;
    }
    
    ASSERT(agent_actor_);
    agent_actor_ = nullptr;

    ASSERT(goal_actor_);
    goal_actor_ = nullptr;
}

std::map<std::string, Box> OpenBotAgentController::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    Box box;
    
    box.low = -1.f;
    box.high = 1.f;
    box.shape = {2};
    box.dtype = DataType::Float32;
    action_space["apply_voltage"] = std::move(box);

    return action_space;
}

std::map<std::string, Box> OpenBotAgentController::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    Box box;
    
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {5};
        box.dtype = DataType::Float32;
        observation_space["physical_observation"] = std::move(box);

        box.low = 0;
        box.high = 255;
        box.shape = {Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"}),
                     Config::getValue<unsigned long>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}),
                     3};
        box.dtype = DataType::UInteger8;
        observation_space["visual_observation"] = std::move(box);
    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "physical") {
        box.low = std::numeric_limits<float>::lowest();
        box.high = std::numeric_limits<float>::max();
        box.shape = {5};
        box.dtype = DataType::Float32;
        observation_space["physical_observation"] = std::move(box);
    } else {
        ASSERT(false);
    }

    return observation_space;
}

void OpenBotAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    ASSERT(action.count("apply_voltage"));
    ASSERT(isfinite(action.at("apply_voltage").at(0)));
    ASSERT(isfinite(action.at("apply_voltage").at(1)));

    // @TODO: This can be checked in python?
    ASSERT(action.at("apply_voltage").at(0) >= getActionSpace()["apply_voltage"].low && action.at("apply_voltage").at(0) <= getActionSpace()["apply_voltage"].high, "%f", action.at("apply_voltage").at(0));
    ASSERT(action.at("apply_voltage").at(1) >= getActionSpace()["apply_voltage"].low && action.at("apply_voltage").at(1) <= getActionSpace()["apply_voltage"].high, "%f", action.at("apply_voltage").at(1));

    ASimpleVehiclePawn* vehicle_pawn = dynamic_cast<ASimpleVehiclePawn*>(agent_actor_);
    ASSERT(vehicle_pawn);
    vehicle_pawn->MoveLeftRight(action.at("apply_voltage").at(0), action.at("apply_voltage").at(1));
}

std::map<std::string, std::vector<uint8_t>> OpenBotAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    // get observations
    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "OBSERVATION_MODE"}) == "mixed") {
        ASSERT(IsInGameThread());

        FTextureRenderTargetResource* target_resource = scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
        ASSERT(target_resource);

        TArray<FColor> pixels;

        struct FReadSurfaceContext
        {
            FRenderTarget* src_render_target_;
            TArray<FColor>& out_data_;
            FIntRect rect_;
            FReadSurfaceDataFlags flags_;
        };

        FReadSurfaceContext context = {target_resource, pixels, FIntRect(0, 0, target_resource->GetSizeXY().X, target_resource->GetSizeXY().Y), FReadSurfaceDataFlags(RCM_UNorm, CubeFace_MAX)};

        // Required for uint8 read mode
        context.flags_.SetLinearToGamma(false);

        ENQUEUE_RENDER_COMMAND(ReadSurfaceCommand)([context](FRHICommandListImmediate& RHICmdList) {
            RHICmdList.ReadSurfaceData(context.src_render_target_->GetRenderTargetTexture(), context.rect_, context.out_data_, context.flags_);
        });

        FRenderCommandFence ReadPixelFence;
        ReadPixelFence.BeginFence(true);
        ReadPixelFence.Wait(true);


        std::vector<uint8_t> image(Config::getValue<int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_HEIGHT"}) *
                                   Config::getValue<int>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "MIXED_MODE", "IMAGE_WIDTH"}) *
                                   3);

        for (uint32 i = 0; i < static_cast<uint32>(pixels.Num()); ++i) {
            image.at(3 * i + 0) = pixels[i].R;
            image.at(3 * i + 1) = pixels[i].G;
            image.at(3 * i + 2) = pixels[i].B;
        }
        
        observation["visual_observation"] = std::move(image);
    } 
    
    const FVector agent_current_location = agent_actor_->GetActorLocation();
    const FRotator agent_current_orientation = agent_actor_->GetActorRotation();

    if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "PHYSICAL_OBSERVATION_MODE"}) == "dist-sin-cos") {
        // Get relative position to the goal in the global coordinate system:
        const FVector2D relative_position_to_goal((goal_actor_->GetActorLocation() - agent_current_location).X, (goal_actor_->GetActorLocation() - agent_current_location).Y);

        // Compute Euclidean distance to target:
        float mag_relative_position_to_goal = relative_position_to_goal.Size();

        // Compute robot forward axis (global coordinate system)
        FVector forward_axis = FVector(1.f, 0.f, 0.f); // Front axis is the X axis.
        FVector forward_axis_rotated = agent_current_orientation.RotateVector(forward_axis);

        // Compute yaw in [rad]:
        float delta_yaw = std::atan2f(forward_axis_rotated.Y, forward_axis_rotated.X) - std::atan2f(relative_position_to_goal.Y, relative_position_to_goal.X);

        // Fit to range [-pi, pi]:
        if (delta_yaw > PI) {
            delta_yaw -= 2 * PI;
        } else if (delta_yaw <= -PI) {
            delta_yaw += 2 * PI;
        }

        // Check the actual OpenBot code:
        // https://github.com/isl-org/OpenBot/blob/7868c54742f8ba3df0ba2a886247a753df982772/android/app/src/main/java/org/openbot/pointGoalNavigation/PointGoalNavigationFragment.java#L103
        float sin_yaw = std::sinf(delta_yaw);
        float cos_yaw = std::cosf(delta_yaw);

        // Fuses the actions received from the python client with those received from the keyboard interface (if this interface is activated in the settings.json file)
        ASimpleVehiclePawn* vehicle_pawn = dynamic_cast<ASimpleVehiclePawn*>(agent_actor_);
        ASSERT(vehicle_pawn);
        Eigen::Vector2f control_state = vehicle_pawn->GetControlState();
        observation["physical_observation"] = Serialize::toUint8(std::vector<float>{control_state(0), control_state(1), mag_relative_position_to_goal, sin_yaw, cos_yaw});

    } else if (Config::getValue<std::string>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "PHYSICAL_OBSERVATION_MODE"}) == "yaw-x-y") {
        ASimpleVehiclePawn* vehicle_pawn = dynamic_cast<ASimpleVehiclePawn*>(agent_actor_);
        ASSERT(vehicle_pawn);
        Eigen::Vector2f control_state = vehicle_pawn->GetControlState();
        observation["physical_observation"] = Serialize::toUint8(std::vector<float>{control_state(0), control_state(1), FMath::DegreesToRadians(agent_current_orientation.Yaw), agent_current_location.X, agent_current_location.Y});

    } else {
        ASSERT(false);
    }
    
    return observation;
}

void OpenBotAgentController::reset()
{
    ASimpleVehiclePawn* vehicle_pawn = dynamic_cast<ASimpleVehiclePawn*>(agent_actor_);
    ASSERT(vehicle_pawn);

    ASSERT(agent_actor_);
    const FVector agent_location = agent_actor_->GetActorLocation();
    vehicle_pawn->TeleportToLocation(agent_location, FQuat(FRotator(0)), true);

    USimpleWheeledVehicleMovementComponent* vehicle_movement_component = dynamic_cast<USimpleWheeledVehicleMovementComponent*>(vehicle_pawn->GetVehicleMovementComponent());
    ASSERT(vehicle_movement_component);

    PxRigidDynamic* rigid_body_dynamic_actor = vehicle_movement_component->PVehicle->getRigidDynamicActor();
    ASSERT(rigid_body_dynamic_actor);
    
    // We want to reset the physics state of OpenBot, so we are inlining the below code from
    // Engine/Source/ThirdParty/PhysX3/PhysX_3.4/Source/PhysXVehicle/src/PxVehicleDrive.cpp::setToRestState(), and
    // Engine/Source/ThirdParty/PhysX3/PhysX_3.4/Source/PhysXVehicle/src/PxVehicleWheels.cpp::setToRestState(), because these functions are protected.
    if(!(rigid_body_dynamic_actor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) {
		rigid_body_dynamic_actor->setLinearVelocity(PxVec3(0,0,0));
		rigid_body_dynamic_actor->setAngularVelocity(PxVec3(0,0,0));
		rigid_body_dynamic_actor->clearForce(PxForceMode::eACCELERATION);
		rigid_body_dynamic_actor->clearForce(PxForceMode::eVELOCITY_CHANGE);
		rigid_body_dynamic_actor->clearTorque(PxForceMode::eACCELERATION);
		rigid_body_dynamic_actor->clearTorque(PxForceMode::eVELOCITY_CHANGE);
	}
    vehicle_movement_component->PVehicle->mWheelsDynData.setToRestState();

    // PVehicleDrive is not intiliazed, and so PVehicleDrive->mDriveDynData.setToRestState() is commented out. We want to know if this changes at some point.
    ASSERT(!vehicle_movement_component->PVehicleDrive);
    // vehicle_movement_component->PVehicleDrive->mDriveDynData.setToRestState(); // throws seg fault
}

bool OpenBotAgentController::isReady() const
{
    ASSERT(agent_actor_);
    return agent_actor_->GetVelocity().Size() < Config::getValue<float>({"SIMULATION_CONTROLLER", "OPENBOT_AGENT_CONTROLLER", "AGENT_READY_VELOCITY_THRESHOLD"}); 
}
