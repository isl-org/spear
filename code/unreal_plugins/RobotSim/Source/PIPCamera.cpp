#include "PIPCamera.h"
#include "Runtime/CoreUObject/Public/UObject/ConstructorHelpers.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Camera/CameraComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "ImageUtils.h"

#include <string>
#include <exception>
#include "RobotBlueprintLib.h"


APIPCamera::APIPCamera()
{
    static ConstructorHelpers::FObjectFinder<UMaterial> mat_finder(TEXT("Material'/RobotSim/HUDAssets/CameraSensorNoise.CameraSensorNoise'"));
    if (mat_finder.Succeeded())
    {
        noise_material_static_ = mat_finder.Object;
    }
    else
        URobotBlueprintLib::LogMessageString("Cannot create noise material for the PIPCamera", 
            "", LogDebugLevel::Failure);

    PrimaryActorTick.bCanEverTick = true;
}

void APIPCamera::PostInitializeComponents()
{
    Super::PostInitializeComponents();

    camera_ = URobotBlueprintLib::GetActorComponent<UCameraComponent>(this, TEXT("CameraComponent"));
    captures_.Init(nullptr, imageTypeCount());
    render_targets_.Init(nullptr, imageTypeCount());

    captures_[Utils::toNumeric(ImageType::Scene)] = 
        URobotBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SceneCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthPlanner)] = 
        URobotBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthPlannerCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthPerspective)] =
        URobotBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthPerspectiveCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DepthVis)] = 
        URobotBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DepthVisCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::DisparityNormalized)] = 
        URobotBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("DisparityNormalizedCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::Segmentation)] = 
        URobotBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("SegmentationCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::Infrared)] = 
        URobotBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("InfraredCaptureComponent"));
    captures_[Utils::toNumeric(ImageType::SurfaceNormals)] = 
        URobotBlueprintLib::GetActorComponent<USceneCaptureComponent2D>(this, TEXT("NormalsCaptureComponent"));
}

void APIPCamera::BeginPlay()
{
    Super::BeginPlay();
	int ImageCount = imageTypeCount();
    noise_materials_.AddZeroed(imageTypeCount() + 1);

    //by default all image types are disabled
    camera_type_enabled_.assign(imageTypeCount(), false);

    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        //use final color for all calculations
        captures_[image_type]->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;

        render_targets_[image_type] = NewObject<UTextureRenderTarget2D>();
    }

    gimbal_stabilization_ = 0;
    gimbald_rotator_ = this->GetActorRotation();
    this->SetActorTickEnabled(false);
}

RobotSim::ProjectionMatrix APIPCamera::getProjectionMatrix(const APIPCamera::ImageType image_type) const
{
    //TODO: avoid the need to override const cast here
    const_cast<APIPCamera*>(this)->setCameraTypeEnabled(image_type, true);
    const USceneCaptureComponent2D* capture = const_cast<APIPCamera*>(this)->getCaptureComponent(image_type, false);
    if (capture) {
        FMatrix proj_mat;

	    float x_axis_multiplier;
	    float y_axis_multiplier;
		FIntPoint render_target_size(capture->TextureTarget->GetSurfaceWidth(), capture->TextureTarget->GetSurfaceHeight());

	    if (render_target_size.X > render_target_size.Y)
	    {
		    // if the viewport is wider than it is tall
		    x_axis_multiplier = 1.0f;
		    y_axis_multiplier = render_target_size.X / static_cast<float>(render_target_size.Y);
	    }
	    else
	    {
		    // if the viewport is taller than it is wide
		    x_axis_multiplier = render_target_size.Y / static_cast<float>(render_target_size.X);
		    y_axis_multiplier = 1.0f;
	    }

	    if (capture->ProjectionType == ECameraProjectionMode::Orthographic)
	    {
		    check((int32)ERHIZBuffer::IsInverted);
		    const float OrthoWidth = capture->OrthoWidth / 2.0f;
		    const float OrthoHeight = capture->OrthoWidth / 2.0f * x_axis_multiplier / y_axis_multiplier;

		    const float NearPlane = 0;
		    const float FarPlane = WORLD_MAX / 8.0f;

		    const float ZScale = 1.0f / (FarPlane - NearPlane);
		    const float ZOffset = -NearPlane;

		    proj_mat = FReversedZOrthoMatrix(
			    OrthoWidth,
			    OrthoHeight,
			    ZScale,
			    ZOffset
			    );
	    }
	    else
	    {
            float fov = Utils::degreesToRadians(capture->FOVAngle);
		    if ((int32)ERHIZBuffer::IsInverted)
		    {
			    proj_mat = FReversedZPerspectiveMatrix(
				    fov,
				    fov,
				    x_axis_multiplier,
				    y_axis_multiplier,
				    GNearClippingPlane,
				    GNearClippingPlane
				    );
		    }
		    else
		    {
			    proj_mat = FPerspectiveMatrix(
				    fov,
				    fov,
				    x_axis_multiplier,
				    y_axis_multiplier,
				    GNearClippingPlane,
				    GNearClippingPlane
				    );
		    }
	    }
        RobotSim::ProjectionMatrix mat;
        for (auto i = 0; i < 4; ++i)
            for (auto j = 0; j < 4; ++j)
                mat.matrix[i][j] = proj_mat.M[i][j];
        return mat;
    }
    else {
        RobotSim::ProjectionMatrix mat;
        mat.setTo(Utils::nan<float>());
        return mat;
    }
}

void APIPCamera::Tick(float DeltaTime)
{
    if (gimbal_stabilization_ > 0) {
        FRotator rotator = this->GetActorRotation();
        if (!std::isnan(gimbald_rotator_.Pitch))
            rotator.Pitch = gimbald_rotator_.Pitch * gimbal_stabilization_ + 
                rotator.Pitch * (1 - gimbal_stabilization_);
        if (!std::isnan(gimbald_rotator_.Roll))
            rotator.Roll = gimbald_rotator_.Roll * gimbal_stabilization_ +
                rotator.Roll * (1 - gimbal_stabilization_);
        if (!std::isnan(gimbald_rotator_.Yaw))
            rotator.Yaw = gimbald_rotator_.Yaw * gimbal_stabilization_ + 
                rotator.Yaw * (1 - gimbal_stabilization_);

        this->SetActorRotation(rotator);
    }
}

void APIPCamera::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (noise_materials_.Num()) {
        for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
            if (noise_materials_[image_type + 1])
                captures_[image_type]->PostProcessSettings.RemoveBlendable(noise_materials_[image_type + 1]);
        }
        if (noise_materials_[0])
            camera_->PostProcessSettings.RemoveBlendable(noise_materials_[0]);
    }

    noise_material_static_ = nullptr;
    noise_materials_.Empty();

    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        //use final color for all calculations
        captures_[image_type] = nullptr;
        render_targets_[image_type] = nullptr;
    }
}

unsigned int APIPCamera::imageTypeCount()
{
    return Utils::toNumeric(ImageType::Count);
}

void APIPCamera::showToScreen()
{
    camera_->SetVisibility(true);
    camera_->Activate();
    APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    controller->SetViewTarget(this);
    URobotBlueprintLib::LogMessage(TEXT("Camera: "), GetName(), LogDebugLevel::Informational);
}

void APIPCamera::disableAll()
{
    disableMain();
    disableAllPIP();
}

bool APIPCamera::getCameraTypeEnabled(ImageType type) const
{
    return camera_type_enabled_[Utils::toNumeric(type)];
}

void APIPCamera::setCameraTypeEnabled(ImageType type, bool enabled)
{
    enableCaptureComponent(type, enabled);
}

void APIPCamera::setCameraOrientation(const FRotator& rotator)
{
    if (gimbal_stabilization_ > 0) {
        gimbald_rotator_.Pitch = rotator.Pitch;
        gimbald_rotator_.Roll = rotator.Roll;
        gimbald_rotator_.Yaw = rotator.Yaw;
    }
    this->SetActorRelativeRotation(rotator);
}

void APIPCamera::setupCameraFromSettings(const APIPCamera::CameraSetting& camera_setting, const NedTransform& ned_transform)
{
    //TODO: should we be ignoring position and orientation settings here?

    //TODO: can we eliminate storing NedTransform?
    ned_transform_ = &ned_transform;

    gimbal_stabilization_ = Utils::clip(camera_setting.gimbal.stabilization, 0.0f, 1.0f);
    if (gimbal_stabilization_ > 0) {
        this->SetActorTickEnabled(true);
        gimbald_rotator_.Pitch = camera_setting.gimbal.rotation.pitch;
        gimbald_rotator_.Roll = camera_setting.gimbal.rotation.roll;
        gimbald_rotator_.Yaw = camera_setting.gimbal.rotation.yaw;
    }
    else
        this->SetActorTickEnabled(false);

    int image_count = static_cast<int>(Utils::toNumeric(ImageType::Count));
    for (int image_type = -1; image_type < image_count; ++image_type) {
        const auto& capture_setting = camera_setting.capture_settings.at(image_type);
        const auto& noise_setting = camera_setting.noise_settings.at(image_type);

        if (image_type >= 0) { //scene capture components
            updateCaptureComponentSetting(captures_[image_type], render_targets_[image_type],
                capture_setting, ned_transform);

            setNoiseMaterial(image_type, captures_[image_type], captures_[image_type]->PostProcessSettings, noise_setting);
        }
        else { //camera component
            updateCameraSetting(camera_, capture_setting, ned_transform);

            setNoiseMaterial(image_type, camera_, camera_->PostProcessSettings, noise_setting);
        }
    }
}

void APIPCamera::updateCaptureComponentSetting(USceneCaptureComponent2D* capture, UTextureRenderTarget2D* render_target, 
    const CaptureSetting& setting, const NedTransform& ned_transform)
{
    render_target->InitAutoFormat(setting.width, setting.height); //256 X 144, X 480
    if (!std::isnan(setting.target_gamma))
        render_target->TargetGamma = setting.target_gamma;

    capture->ProjectionType = static_cast<ECameraProjectionMode::Type>(setting.projection_mode);

    if (!std::isnan(setting.fov_degrees))
        capture->FOVAngle = setting.fov_degrees;
    if (capture->ProjectionType == ECameraProjectionMode::Orthographic && !std::isnan(setting.ortho_width))
        capture->OrthoWidth = ned_transform.fromNed(setting.ortho_width);

    updateCameraPostProcessingSetting(capture->PostProcessSettings, setting);
}

void APIPCamera::updateCameraSetting(UCameraComponent* camera, const CaptureSetting& setting, const NedTransform& ned_transform)
{
    //if (!std::isnan(setting.target_gamma))
    //    camera-> = setting.target_gamma;

    camera->SetProjectionMode(static_cast<ECameraProjectionMode::Type>(setting.projection_mode));

    if (!std::isnan(setting.fov_degrees))
        camera->SetFieldOfView(setting.fov_degrees);
    if (camera->ProjectionMode == ECameraProjectionMode::Orthographic && !std::isnan(setting.ortho_width))
        camera->SetOrthoWidth(ned_transform.fromNed(setting.ortho_width));

    updateCameraPostProcessingSetting(camera->PostProcessSettings, setting);
}

RobotSim::Pose APIPCamera::getPose() const
{
    return ned_transform_->toLocalNed(this->GetActorTransform());
}

void APIPCamera::updateCameraPostProcessingSetting(FPostProcessSettings& obj, const CaptureSetting& setting)
{
    if (!std::isnan(setting.motion_blur_amount))
        obj.MotionBlurAmount = setting.motion_blur_amount;
    if (setting.auto_exposure_method >= 0)
        obj.AutoExposureMethod = Utils::toEnum<EAutoExposureMethod>(setting.auto_exposure_method);       
    if (!std::isnan(setting.auto_exposure_speed))
        obj.AutoExposureSpeedDown = obj.AutoExposureSpeedUp = setting.auto_exposure_speed;
    if (!std::isnan(setting.auto_exposure_max_brightness))
        obj.AutoExposureMaxBrightness = setting.auto_exposure_max_brightness;
    if (!std::isnan(setting.auto_exposure_min_brightness))
        obj.AutoExposureMinBrightness = setting.auto_exposure_min_brightness;
    if (!std::isnan(setting.auto_exposure_bias))
        obj.AutoExposureBias = setting.auto_exposure_bias;
    if (!std::isnan(setting.auto_exposure_low_percent))
        obj.AutoExposureLowPercent = setting.auto_exposure_low_percent;        
    if (!std::isnan(setting.auto_exposure_high_percent))
        obj.AutoExposureHighPercent = setting.auto_exposure_high_percent;    
    if (!std::isnan(setting.auto_exposure_histogram_log_min))
        obj.HistogramLogMin = setting.auto_exposure_histogram_log_min;    
    if (!std::isnan(setting.auto_exposure_histogram_log_max))
        obj.HistogramLogMax = setting.auto_exposure_histogram_log_max;  
}

void APIPCamera::setNoiseMaterial(int image_type, UObject* outer, FPostProcessSettings& obj, const NoiseSetting& settings)
{
    if (!settings.Enabled)
        return;

    UMaterialInstanceDynamic* noise_material_ = UMaterialInstanceDynamic::Create(noise_material_static_, outer);
    noise_materials_[image_type + 1] = noise_material_;


    noise_material_->SetScalarParameterValue("HorzWaveStrength", settings.HorzWaveStrength);
    noise_material_->SetScalarParameterValue("RandSpeed", settings.RandSpeed);
    noise_material_->SetScalarParameterValue("RandSize", settings.RandSize);
    noise_material_->SetScalarParameterValue("RandDensity", settings.RandDensity);
    noise_material_->SetScalarParameterValue("RandContrib", settings.RandContrib);
    noise_material_->SetScalarParameterValue("HorzWaveContrib", settings.HorzWaveContrib);
    noise_material_->SetScalarParameterValue("HorzWaveVertSize", settings.HorzWaveVertSize);
    noise_material_->SetScalarParameterValue("HorzWaveScreenSize", settings.HorzWaveScreenSize);
    noise_material_->SetScalarParameterValue("HorzNoiseLinesContrib", settings.HorzNoiseLinesContrib);
    noise_material_->SetScalarParameterValue("HorzNoiseLinesDensityY", settings.HorzNoiseLinesDensityY);
    noise_material_->SetScalarParameterValue("HorzNoiseLinesDensityXY", settings.HorzNoiseLinesDensityXY);
    noise_material_->SetScalarParameterValue("HorzDistortionStrength", settings.HorzDistortionStrength);
    noise_material_->SetScalarParameterValue("HorzDistortionContrib", settings.HorzDistortionContrib);

    obj.AddBlendable(noise_material_, 1.0f);
}

void APIPCamera::enableCaptureComponent(const APIPCamera::ImageType type, bool is_enabled)
{
    USceneCaptureComponent2D* capture = getCaptureComponent(type, false);
    if (capture != nullptr) {
        if (is_enabled) {
            //do not make unnecessary calls to Activate() which otherwise causes crash in Unreal
            if (!capture->IsActive() || capture->TextureTarget == nullptr) {
                capture->TextureTarget = getRenderTarget(type, false);
                capture->Activate();
            }
        }
        else {
            if (capture->IsActive() || capture->TextureTarget != nullptr) {
                capture->Deactivate();
                capture->TextureTarget = nullptr;
            }
        }
        camera_type_enabled_[Utils::toNumeric(type)] = is_enabled;
    }
    //else nothing to enable
}

UTextureRenderTarget2D* APIPCamera::getRenderTarget(const APIPCamera::ImageType type, bool if_active)
{
    unsigned int image_type = Utils::toNumeric(type);

    if (!if_active || camera_type_enabled_[image_type])
        return render_targets_[image_type];
    return nullptr;
}

USceneCaptureComponent2D* APIPCamera::getCaptureComponent(const APIPCamera::ImageType type, bool if_active)
{
    unsigned int image_type = Utils::toNumeric(type);

    if (!if_active || camera_type_enabled_[image_type])
        return captures_[image_type];
    return nullptr;
}

void APIPCamera::disableAllPIP()
{
    for (unsigned int image_type = 0; image_type < imageTypeCount(); ++image_type) {
        enableCaptureComponent(Utils::toEnum<ImageType>(image_type), false);
    }
}


void APIPCamera::disableMain()
{
    camera_->Deactivate();
    camera_->SetVisibility(false);
    //APlayerController* controller = this->GetWorld()->GetFirstPlayerController();
    //if (controller && controller->GetViewTarget() == this)
    //    controller->SetViewTarget(nullptr);
}




