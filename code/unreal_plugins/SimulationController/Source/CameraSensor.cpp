#include "CameraSensor.h"

#include <string>
#include <utility>
#include <vector>

#include <Components/SceneCaptureComponent2D.h>
#include <Engine/TextureRenderTarget2D.h>
#include "Kismet/KismetSystemLibrary.h"
#include "Materials/MaterialInstanceDynamic.h"
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <UObject/UObjectGlobals.h>

#include "Assert.h"
#include "Config.h"
#include "Serialize.h"

CameraSensor::CameraSensor(UWorld* world, AActor* actor_){
        camera_actor_ = actor_;
        ASSERT(camera_actor_);

        new_object_parent_actor_ = world->SpawnActor<AActor>();
        ASSERT(new_object_parent_actor_);

        // create SceneCaptureComponent2D and TextureRenderTarget2D
        this->scene_capture_component_ = NewObject<USceneCaptureComponent2D>(new_object_parent_actor_, TEXT("SceneCaptureComponent2D"));
        ASSERT(scene_capture_component_);       
        this->scene_capture_component_->AttachToComponent(camera_actor_->GetRootComponent(), FAttachmentTransformRules::SnapToTargetNotIncludingScale);
        this->scene_capture_component_->SetVisibility(true);
        this->scene_capture_component_->CaptureSource = ESceneCaptureSource::SCS_FinalColorLDR;
        this->scene_capture_component_->FOVAngle = 60.f;
        //scene_capture_component_->ShowFlags.SetTemporalAA(false);

        SetCameraDefaultOverrides();    
        ConfigureShowFlags(this->enable_postprocessing_effects_); 

        UKismetSystemLibrary::ExecuteConsoleCommand(world, FString("g.TimeoutForBlockOnRenderFence 300000"));

        this->texture_render_target_ = NewObject<UTextureRenderTarget2D>(new_object_parent_actor_, TEXT("TextureRenderTarget2D"));
        ASSERT(texture_render_target_);     
}

CameraSensor::~CameraSensor(){
    ASSERT(this->texture_render_target_);
    this->texture_render_target_->MarkPendingKill();
    this->texture_render_target_ = nullptr;

    ASSERT(this->scene_capture_component_);
    this->scene_capture_component_->DestroyComponent();
    this->scene_capture_component_ = nullptr;

    ASSERT(this->new_object_parent_actor_);
    this->new_object_parent_actor_->Destroy();
    this->new_object_parent_actor_ = nullptr;

    ASSERT(this->camera_actor_);
    this->camera_actor_ = nullptr;
}

void CameraSensor::SetRenderTarget(unsigned long w, unsigned long h){
        this->texture_render_target_->InitCustomFormat(w ,h ,PF_B8G8R8A8 ,true ); // PF_B8G8R8A8 disables HDR;
        this->texture_render_target_->RenderTargetFormat = ETextureRenderTargetFormat::RTF_RGBA8;
        this->texture_render_target_->bGPUSharedFlag = true; // demand buffer on GPU - might improve performance?
        this->texture_render_target_->TargetGamma = 1;
        this->texture_render_target_->SRGB = false; // false for pixels to be stored in linear space
        this->texture_render_target_->bAutoGenerateMips = false;
        this->texture_render_target_->UpdateResourceImmediate(true);    
        this->scene_capture_component_->TextureTarget = texture_render_target_;
        this->scene_capture_component_->RegisterComponent();
}

void CameraSensor::SetPostProcessingMaterial(const FString &Path){
        UMaterial* mat = LoadObject<UMaterial>(nullptr, *Path);
        ASSERT(mat);
}

void CameraSensor::SetPostProcessBlendable(UMaterial* mat){
	ASSERT(mat);
	this->scene_capture_component_->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(mat, this->scene_capture_component_), 1.0f);
}

void CameraSensor::SetPostProcessBlendables(std::vector<passes> blendables){
        for(passes pass : blendables){
                UMaterial* mat = LoadObject<UMaterial>(nullptr, *PASS_PATHS_[pass]);
                ASSERT(mat);
                this->scene_capture_component_->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(mat, this->scene_capture_component_), 1.0f);
        }
}

void CameraSensor::SetPostProcessBlendables(std::vector<std::string> blendables){
        //Parse blendable names into enum passes
        std::vector<passes> blendable_passes_;
        for(std::string pass_name_ : blendables){
            if(pass_name_ == "depth"){
                blendable_passes_.push_back(passes::Depth);
            }else if(pass_name_ == "segmentation"){
                blendable_passes_.push_back(passes::Segmentation);
            }
        }
        //sort passes
        std::sort(blendable_passes_.begin(), blendable_passes_.end());
        //Set blendables
        for(passes pass : blendable_passes_){
                UMaterial* mat = LoadObject<UMaterial>(nullptr, *PASS_PATHS_[pass]);
                ASSERT(mat);
                this->scene_capture_component_->PostProcessSettings.AddBlendable(UMaterialInstanceDynamic::Create(mat, this->scene_capture_component_), 0.0f);
        } 
}

void CameraSensor::ActivateBlendablePass(passes pass_){
        for(auto pass : this->scene_capture_component_->PostProcessSettings.WeightedBlendables.Array){
                pass.Weight = .0f;
        }
        printf("num of weightedBlendables: %d ", this->scene_capture_component_->PostProcessSettings.WeightedBlendables.Array.Num());
        printf("input pass : %d ", int(pass_));

        if(pass_ != passes::Any){
                this->scene_capture_component_->PostProcessSettings.WeightedBlendables.Array[int(pass_)].Weight = 1.0f;
        }
}

void CameraSensor::ActivateBlendablePass(std::string pass_name){
        //SEARCH THE BEST WAY TO PARSE DIFERENT PASSES FFROM PYTHON AND GIVE THE HAB TO GIVE CUSTOM PASSES FROM CLIENT
}

TArray<FColor> CameraSensor::GetRenderData(){
        FTextureRenderTargetResource* target_resource = this->scene_capture_component_->TextureTarget->GameThread_GetRenderTargetResource();
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
        
        return pixels;
}

void CameraSensor::SetCameraDefaultOverrides(){
	this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureMethod = true;
        this->scene_capture_component_->PostProcessSettings.AutoExposureMethod = EAutoExposureMethod::AEM_Histogram;
        this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureBias = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureMinBrightness = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureMaxBrightness = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureSpeedUp = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureSpeedDown = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_AutoExposureCalibrationConstant_DEPRECATED = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_HistogramLogMin = true;
        this->scene_capture_component_->PostProcessSettings.HistogramLogMin = 1.0f;
        this->scene_capture_component_->PostProcessSettings.bOverride_HistogramLogMax = true;
        this->scene_capture_component_->PostProcessSettings.HistogramLogMax = 12.0f;

        // Camera
        this->scene_capture_component_->PostProcessSettings.bOverride_CameraShutterSpeed = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_CameraISO = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_DepthOfFieldFstop = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_DepthOfFieldMinFstop = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_DepthOfFieldBladeCount = true;

        // Film (Tonemapper)
        this->scene_capture_component_->PostProcessSettings.bOverride_FilmSlope = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_FilmToe = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_FilmShoulder = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_FilmWhiteClip = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_FilmBlackClip = true;

        // Motion blur
        this->scene_capture_component_->PostProcessSettings.bOverride_MotionBlurAmount = true;
        this->scene_capture_component_->PostProcessSettings.MotionBlurAmount = 0.45f;
        this->scene_capture_component_->PostProcessSettings.bOverride_MotionBlurMax = true;
        this->scene_capture_component_->PostProcessSettings.MotionBlurMax = 0.35f;
        this->scene_capture_component_->PostProcessSettings.bOverride_MotionBlurPerObjectSize = true;
        this->scene_capture_component_->PostProcessSettings.MotionBlurPerObjectSize = 0.1f;

        // Color Grading
        this->scene_capture_component_->PostProcessSettings.bOverride_WhiteTemp = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_WhiteTint = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_ColorContrast = true;
#if PLATFORM_LINUX
        // Looks like Windows and Linux have different outputs with the
        // same exposure compensation, this fixes it.
        this->scene_capture_component_->PostProcessSettings.ColorContrast = FVector4(1.2f, 1.2f, 1.2f, 1.0f);
#endif

        // Chromatic Aberration
        this->scene_capture_component_->PostProcessSettings.bOverride_SceneFringeIntensity = true;
        this->scene_capture_component_->PostProcessSettings.bOverride_ChromaticAberrationStartOffset = true;

        // Ambient Occlusion
        this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionIntensity = true;
        this->scene_capture_component_->PostProcessSettings.AmbientOcclusionIntensity = 0.5f;
        this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionRadius = true;
        this->scene_capture_component_->PostProcessSettings.AmbientOcclusionRadius = 100.0f;
        this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionStaticFraction = true;
        this->scene_capture_component_->PostProcessSettings.AmbientOcclusionStaticFraction = 1.0f;
        this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionFadeDistance = true;
        this->scene_capture_component_->PostProcessSettings.AmbientOcclusionFadeDistance = 50000.0f;
        this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionPower = true;
        this->scene_capture_component_->PostProcessSettings.AmbientOcclusionPower = 2.0f;
        this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionBias = true;
        this->scene_capture_component_->PostProcessSettings.AmbientOcclusionBias = 3.0f;
        this->scene_capture_component_->PostProcessSettings.bOverride_AmbientOcclusionQuality = true;
        this->scene_capture_component_->PostProcessSettings.AmbientOcclusionQuality = 100.0f;

        // Bloom
        this->scene_capture_component_->PostProcessSettings.bOverride_BloomMethod = true;
        this->scene_capture_component_->PostProcessSettings.BloomMethod = EBloomMethod::BM_SOG;
        this->scene_capture_component_->PostProcessSettings.bOverride_BloomIntensity = true;
        this->scene_capture_component_->PostProcessSettings.BloomIntensity = 0.675f;
        this->scene_capture_component_->PostProcessSettings.bOverride_BloomThreshold = true;
        this->scene_capture_component_->PostProcessSettings.BloomThreshold = -1.0f;

        // Lens
        this->scene_capture_component_->PostProcessSettings.bOverride_LensFlareIntensity = true;
        this->scene_capture_component_->PostProcessSettings.LensFlareIntensity = 0.1;

}

void CameraSensor::ConfigureShowFlags(bool bPostProcessing){
	if (bPostProcessing)
        {
            this->scene_capture_component_->ShowFlags.EnableAdvancedFeatures();
            this->scene_capture_component_->ShowFlags.SetMotionBlur(true);
            return;
        }

        this->scene_capture_component_->ShowFlags.SetAmbientOcclusion(false);
        this->scene_capture_component_->ShowFlags.SetAntiAliasing(false);
        this->scene_capture_component_->ShowFlags.SetVolumetricFog(false);
        // ShowFlags.SetAtmosphericFog(false);
        // ShowFlags.SetAudioRadius(false);
        // ShowFlags.SetBillboardSprites(false);
        this->scene_capture_component_->ShowFlags.SetBloom(false);
        // ShowFlags.SetBounds(false);
        // ShowFlags.SetBrushes(false);
        // ShowFlags.SetBSP(false);
        // ShowFlags.SetBSPSplit(false);
        // ShowFlags.SetBSPTriangles(false);
        // ShowFlags.SetBuilderBrush(false);
        // ShowFlags.SetCameraAspectRatioBars(false);
        // ShowFlags.SetCameraFrustums(false);
        this->scene_capture_component_->ShowFlags.SetCameraImperfections(false);
        this->scene_capture_component_->ShowFlags.SetCameraInterpolation(false);
        // ShowFlags.SetCameraSafeFrames(false);
        // ShowFlags.SetCollision(false);
        // ShowFlags.SetCollisionPawn(false);
        // ShowFlags.SetCollisionVisibility(false);
        this->scene_capture_component_->ShowFlags.SetColorGrading(false);
        // ShowFlags.SetCompositeEditorPrimitives(false);
        // ShowFlags.SetConstraints(false);
        // ShowFlags.SetCover(false);
        // ShowFlags.SetDebugAI(false);
        // ShowFlags.SetDecals(false);
        // ShowFlags.SetDeferredLighting(false);
        this->scene_capture_component_->ShowFlags.SetDepthOfField(false);
        this->scene_capture_component_->ShowFlags.SetDiffuse(false);
        this->scene_capture_component_->ShowFlags.SetDirectionalLights(false);
        this->scene_capture_component_->ShowFlags.SetDirectLighting(false);
        // ShowFlags.SetDistanceCulledPrimitives(false);
        // ShowFlags.SetDistanceFieldAO(false);
        // ShowFlags.SetDistanceFieldGI(false);
        this->scene_capture_component_->ShowFlags.SetDynamicShadows(false);
        // ShowFlags.SetEditor(false);
        this->scene_capture_component_->ShowFlags.SetEyeAdaptation(false);
        this->scene_capture_component_->ShowFlags.SetFog(false);
        // ShowFlags.SetGame(false);
        // ShowFlags.SetGameplayDebug(false);
        // ShowFlags.SetGBufferHints(false);
        this->scene_capture_component_->ShowFlags.SetGlobalIllumination(false);
        this->scene_capture_component_->ShowFlags.SetGrain(false);
        // ShowFlags.SetGrid(false);
        // ShowFlags.SetHighResScreenshotMask(false);
        // ShowFlags.SetHitProxies(false);
        this->scene_capture_component_->ShowFlags.SetHLODColoration(false);
        this->scene_capture_component_->ShowFlags.SetHMDDistortion(false);
        // ShowFlags.SetIndirectLightingCache(false);
        // ShowFlags.SetInstancedFoliage(false);
        // ShowFlags.SetInstancedGrass(false);
        // ShowFlags.SetInstancedStaticMeshes(false);
        // ShowFlags.SetLandscape(false);
        // ShowFlags.SetLargeVertices(false);
        this->scene_capture_component_->ShowFlags.SetLensFlares(false);
        this->scene_capture_component_->ShowFlags.SetLevelColoration(false);
        this->scene_capture_component_->ShowFlags.SetLightComplexity(false);
        this->scene_capture_component_->ShowFlags.SetLightFunctions(false);
        this->scene_capture_component_->ShowFlags.SetLightInfluences(false);
        this->scene_capture_component_->ShowFlags.SetLighting(false);
        this->scene_capture_component_->ShowFlags.SetLightMapDensity(false);
        this->scene_capture_component_->ShowFlags.SetLightRadius(false);
        this->scene_capture_component_->ShowFlags.SetLightShafts(false);
        // ShowFlags.SetLOD(false);
        this->scene_capture_component_->ShowFlags.SetLODColoration(false);
        // ShowFlags.SetMaterials(false);
        // ShowFlags.SetMaterialTextureScaleAccuracy(false);
        // ShowFlags.SetMeshEdges(false);
        // ShowFlags.SetMeshUVDensityAccuracy(false);
        // ShowFlags.SetModeWidgets(false);
        this->scene_capture_component_->ShowFlags.SetMotionBlur(false);
        // ShowFlags.SetNavigation(false);
        this->scene_capture_component_->ShowFlags.SetOnScreenDebug(false);
        // ShowFlags.SetOutputMaterialTextureScales(false);
        // ShowFlags.SetOverrideDiffuseAndSpecular(false);
        // ShowFlags.SetPaper2DSprites(false);
        this->scene_capture_component_->ShowFlags.SetParticles(false);
        // ShowFlags.SetPivot(false);
        this->scene_capture_component_->ShowFlags.SetPointLights(false);
        // ShowFlags.SetPostProcessing(false);
        // ShowFlags.SetPostProcessMaterial(false);
        // ShowFlags.SetPrecomputedVisibility(false);
        // ShowFlags.SetPrecomputedVisibilityCells(false);
        // ShowFlags.SetPreviewShadowsIndicator(false);
        // ShowFlags.SetPrimitiveDistanceAccuracy(false);
        this->scene_capture_component_->ShowFlags.SetPropertyColoration(false);
        // ShowFlags.SetQuadOverdraw(false);
        // ShowFlags.SetReflectionEnvironment(false);
        // ShowFlags.SetReflectionOverride(false);
        this->scene_capture_component_->ShowFlags.SetRefraction(false);
        // ShowFlags.SetRendering(false);
        this->scene_capture_component_->ShowFlags.SetSceneColorFringe(false);
        // ShowFlags.SetScreenPercentage(false);
        this->scene_capture_component_->ShowFlags.SetScreenSpaceAO(false);
        this->scene_capture_component_->ShowFlags.SetScreenSpaceReflections(false);
        // ShowFlags.SetSelection(false);
        // ShowFlags.SetSelectionOutline(false);
        // ShowFlags.SetSeparateTranslucency(false);
        // ShowFlags.SetShaderComplexity(false);
        // ShowFlags.SetShaderComplexityWithQuadOverdraw(false);
        // ShowFlags.SetShadowFrustums(false);
        // ShowFlags.SetSkeletalMeshes(false);
        // ShowFlags.SetSkinCache(false);
        this->scene_capture_component_->ShowFlags.SetSkyLighting(false);
        // ShowFlags.SetSnap(false);
        // ShowFlags.SetSpecular(false);
        // ShowFlags.SetSplines(false);
        this->scene_capture_component_->ShowFlags.SetSpotLights(false);
        // ShowFlags.SetStaticMeshes(false);
        this->scene_capture_component_->ShowFlags.SetStationaryLightOverlap(false);
        // ShowFlags.SetStereoRendering(false);
        // ShowFlags.SetStreamingBounds(false);
        this->scene_capture_component_->ShowFlags.SetSubsurfaceScattering(false);
        // ShowFlags.SetTemporalAA(false);
        // ShowFlags.SetTessellation(false);
        // ShowFlags.SetTestImage(false);
        // ShowFlags.SetTextRender(false);
        // ShowFlags.SetTexturedLightProfiles(false);
        this->scene_capture_component_->ShowFlags.SetTonemapper(false);
        // ShowFlags.SetTranslucency(false);
        // ShowFlags.SetVectorFields(false);
        // ShowFlags.SetVertexColors(false);
        // ShowFlags.SetVignette(false);
        // ShowFlags.SetVisLog(false);
        // ShowFlags.SetVisualizeAdaptiveDOF(false);
        // ShowFlags.SetVisualizeBloom(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeBuffer(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeDistanceFieldAO(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeDOF(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeHDR(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeLightCulling(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeLPV(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeMeshDistanceFields(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeMotionBlur(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeOutOfBoundsPixels(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeSenses(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeShadingModels(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeSSR(false);
        this->scene_capture_component_->ShowFlags.SetVisualizeSSS(false);
        // ShowFlags.SetVolumeLightingSamples(false);
        // ShowFlags.SetVolumes(false);
        // ShowFlags.SetWidgetComponents(false);
        // ShowFlags.SetWireframe(false);

}
