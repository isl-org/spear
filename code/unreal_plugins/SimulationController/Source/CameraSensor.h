#pragma once

#include <map>
#include <string>
#include <vector>

class AActor;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UTickEvent;
class UWorld;

struct CameraPass
{
    USceneCaptureComponent2D* scene_capture_component_ = nullptr;
    UTextureRenderTarget2D* texture_render_target_ = nullptr;
};

class CameraSensor
{
public:
    CameraSensor(AActor* actor, std::vector<std::string> passes, unsigned long w, unsigned long h);
    ~CameraSensor();

    std::map<std::string, TArray<FColor>> GetRenderData();

    static std::vector<float> FColorToFloatImage(TArray<FColor> data);

    std::map<std::string, CameraPass> camera_passes_;

private:
    void SetCameraParameters(USceneCaptureComponent2D* scene_capture_component);
    void SetCameraParametersNonFinalColor(USceneCaptureComponent2D* scene_capture_component);
};
