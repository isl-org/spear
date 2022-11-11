#include "CameraAgentController.h"

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <AI/NavDataGenerator.h>
#include <Camera/CameraActor.h>
#include <Components/SceneCaptureComponent2D.h>
#include <Engine/SpotLight.h>
#include <Engine/World.h>
#include <EngineUtils.h>
#include <GameFramework/Actor.h>
#include <Kismet/GameplayStatics.h>
#include <NavigationSystem.h>
#include <NavMesh/NavMeshBoundsVolume.h>
#include <NavMesh/RecastNavMesh.h>
#include <NavModifierVolume.h>

#include "Assert/Assert.h"
#include "Box.h"
#include "CameraSensor.h"
#include "Config.h"
#include "Serialize.h"

CameraAgentController::CameraAgentController(UWorld* world)
{
    // store ref to world
    world_ = world;

    // create camera sensor
    FActorSpawnParameters spawn_params;
    spawn_params.Name = FName(Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_ACTOR_NAME" }).c_str());
    spawn_params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    camera_actor_ = world_->SpawnActor<ACameraActor>(FVector(0, 0, Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "AGENT_HEIGHT" })), FRotator(0, 0, 0), spawn_params);
    ASSERT(camera_actor_);
    ASSERT(dynamic_cast<ACameraActor*>(camera_actor_));

    camera_sensor_ = std::make_unique<CameraSensor>(
        dynamic_cast<ACameraActor*>(camera_actor_)->GetCameraComponent(),
        Config::getValue<std::vector<std::string>>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "RENDER_PASSES" }),
        Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_WIDTH" }),
        Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_HEIGHT" }));
    ASSERT(camera_sensor_);

    // update camera parameters
    for (auto& camera_pass : camera_sensor_->camera_passes_) {

        camera_pass.second.scene_capture_component_->FOVAngle = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FOV" });

        if (camera_pass.first == "final_color") {
            // update auto-exposure settings
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_AutoExposureSpeedUp       = Config::getValue<bool> ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_AUTO_EXPOSURE_SPEED_UP" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.AutoExposureSpeedUp                 = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_AUTO_EXPOSURE_SPEED_UP" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_AutoExposureSpeedDown     = Config::getValue<bool> ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_AUTO_EXPOSURE_SPEED_DOWN" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.AutoExposureSpeedDown               = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_AUTO_EXPOSURE_SPEED_DOWN" });

            // enable raytracing features
            camera_pass.second.scene_capture_component_->bUseRayTracingIfEnabled = Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_USE_RAYTRACING_IF_ENABLED" });

            // update indirect lighting 
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_IndirectLightingIntensity = Config::getValue<bool> ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_INDIRECT_LIGHTING_INTENSITY" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.IndirectLightingIntensity           = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_INDIRECT_LIGHTING_INTENSITY" });

            // update raytracing global illumination
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_RayTracingGI = Config::getValue<bool>       ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_RAYTRACING_GI" });
            auto raytracing_gi_type = Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_RAYTRACING_GI_TYPE" });
            if (raytracing_gi_type == "BruteForce") {
                camera_pass.second.scene_capture_component_->PostProcessSettings.RayTracingGIType = ERayTracingGlobalIlluminationType::BruteForce;
            } else if (raytracing_gi_type != "") {
                ASSERT(false);
            }

            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_RayTracingGIMaxBounces      = Config::getValue<bool>         ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_RAYTRACING_GI_MAX_BOUNCES" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.RayTracingGIMaxBounces                = Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_RAYTRACING_GI_MAX_BOUNCES" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_RayTracingGISamplesPerPixel = Config::getValue<bool>         ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_RAYTRACING_GI_SAMPLES_PER_PIXEL" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.RayTracingGISamplesPerPixel           = Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_RAYTRACING_GI_SAMPLES_PER_PIXEL" });

            // update raytracing ambient occlusion
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_RayTracingAO                = Config::getValue<bool>         ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_RAYTRACING_AO" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.RayTracingAO                          = Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_RAYTRACING_AO" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_RayTracingAOSamplesPerPixel = Config::getValue<bool>         ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_RAYTRACING_AO_SAMPLES_PER_PIXEL" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.RayTracingAOSamplesPerPixel           = Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_RAYTRACING_AO_SAMPLES_PER_PIXEL" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_RayTracingAOIntensity       = Config::getValue<bool>         ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_RAYTRACING_AO_INTENSITY" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.RayTracingAOIntensity                 = Config::getValue<float>        ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_RAYTRACING_AO_INTENSITY" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_RayTracingAORadius          = Config::getValue<bool>         ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_RAYTRACING_AO_RADIUS" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.RayTracingAORadius                    = Config::getValue<float>        ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_RAYTRACING_AO_RADIUS" });

            // update raytracing reflections
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_ReflectionsType = Config::getValue<bool>       ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_REFLECTIONS_TYPE" });
            auto reflections_type                                                          = Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_REFLECTIONS_TYPE" });
            if (reflections_type == "RayTracing") {
                camera_pass.second.scene_capture_component_->PostProcessSettings.ReflectionsType = EReflectionsType::RayTracing; 
            } else if (reflections_type != "") {
                ASSERT(false); 
            }

            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_RayTracingReflectionsMaxBounces      = Config::getValue<bool>         ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_MAX_BOUNCES" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.RayTracingReflectionsMaxBounces                = Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_RAYTRACING_REFLECTIONS_MAX_BOUNCES" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_RayTracingReflectionsMaxRoughness    = Config::getValue<bool>         ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_MAX_ROUGHNESS" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.RayTracingReflectionsMaxRoughness              = Config::getValue<float>        ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_RAYTRACING_REFLECTIONS_MAX_ROUGHNESS" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_RayTracingReflectionsSamplesPerPixel = Config::getValue<bool>         ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_SAMPLES_PER_PIXEL" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.RayTracingReflectionsSamplesPerPixel           = Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_RAYTRACING_REFLECTIONS_SAMPLES_PER_PIXEL" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.bOverride_RayTracingReflectionsTranslucency    = Config::getValue<bool>         ({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_OVERRIDE_RAYTRACING_REFLECTIONS_TRANSLUCENCY" });
            camera_pass.second.scene_capture_component_->PostProcessSettings.RayTracingReflectionsTranslucency              = Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_RAYTRACING_REFLECTIONS_TRANSLUCENCY" });

            // update show flags
            camera_pass.second.scene_capture_component_->ShowFlags.SetAmbientOcclusion              (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_AMBIENT_OCCLUSION" }));                // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetAntiAliasing                  (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_ANTI_ALIASING" }));                    
            camera_pass.second.scene_capture_component_->ShowFlags.SetCameraImperfections           (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_CAMERA_IMPERFECTIONS" }));             // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetColorGrading                  (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_COLOR_GRADING" }));                    // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetDepthOfField                  (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_DEPTH_OF_FIELD" }));                   // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetDistanceFieldAO               (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_DISTANCE_FIELD_AO" }));                // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetRayTracedDistanceFieldShadows (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_RAYTRACED_DISTANCE_FIELD_SHADOWS" }));
            camera_pass.second.scene_capture_component_->ShowFlags.SetDynamicShadows                (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_DYNAMIC_SHADOWS" }));
            camera_pass.second.scene_capture_component_->ShowFlags.SetEyeAdaptation                 (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_EYE_ADAPTATION" }));                   // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetGrain                         (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_GRAIN" }));                            // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetIndirectLightingCache         (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_INDIRECT_LIGHTING_CACHE" }));          // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetLensFlares                    (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_LENS_FLARES" }));                      // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetLightShafts                   (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_LIGHT_SHAFTS" }));                     // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetScreenSpaceReflections        (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_SCREEN_SPACE_REFLECTIONS" }));         // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetSeparateTranslucency          (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_SEPARATE_TRANSLUCENCY" }));            // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetTemporalAA                    (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_TEMPORAL_AA" }));                      // enabled by EnableAdvancedFeatures();
            camera_pass.second.scene_capture_component_->ShowFlags.SetVignette                      (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "FINAL_COLOR_SHOW_FLAGS_SET_VIGNETTE" }));                         // enabled by EnableAdvancedFeatures();
        }
    }
}

CameraAgentController::~CameraAgentController()
{
    ASSERT(camera_sensor_);
    camera_sensor_ = nullptr;

    ASSERT(camera_actor_);
    camera_actor_->Destroy();
    camera_actor_ = nullptr;

    ASSERT(world_);
    world_ = nullptr;
}

void CameraAgentController::findObjectReferences(UWorld* world)
{
    // HACK: find references to spotlights and remove them
    TArray<AActor*> light_actors;
    UGameplayStatics::GetAllActorsOfClass(world_, ALight::StaticClass(), light_actors);

    for (int i = 0; i < light_actors.Num(); i++) {
        ASpotLight* spot_light = Cast<ASpotLight>(light_actors[i]);
        if (spot_light != nullptr) {
            spot_light->Destroy();
        }
    }

    UNavigationSystemV1* nav_sys = FNavigationSystem::GetCurrent<UNavigationSystemV1>(world_);
    ASSERT(nav_sys);

    FNavAgentProperties agent_properties;
    agent_properties.AgentHeight = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "AGENT_HEIGHT" });
    agent_properties.AgentRadius = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "AGENT_RADIUS" });
    agent_properties.AgentStepHeight = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "AGENT_MAX_STEP_HEIGHT" });

    ANavigationData* nav_data = nav_sys->GetNavDataForProps(agent_properties);
    ASSERT(nav_data);

    nav_mesh_ = Cast<ARecastNavMesh>(nav_data);
    ASSERT(nav_mesh_);

    // build navmesh based on properties from config file
    buildNavMesh(nav_sys);
}

void CameraAgentController::cleanUpObjectReferences()
{
    // unassign nav_mesh_ reference
    nav_mesh_ = nullptr;
}

std::map<std::string, Box> CameraAgentController::getActionSpace() const
{
    std::map<std::string, Box> action_space;
    Box box;

    box.low = std::numeric_limits<uint32_t>::lowest();
    box.high = std::numeric_limits<uint32_t>::max();
    box.shape = { 1 };
    box.dtype = DataType::UInteger32;
    action_space["set_num_random_points"] = std::move(box);

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = { 6 };
    box.dtype = DataType::Float32;
    action_space["set_pose"] = std::move(box); // x,y,z in cms and then p,y,r in degs

    return action_space;
}

std::map<std::string, Box> CameraAgentController::getObservationSpace() const
{
    std::map<std::string, Box> observation_space;
    Box box;

    std::vector<std::string> passes = Config::getValue<std::vector<std::string>>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "RENDER_PASSES" });
    for (const auto& pass : passes) {
        box.low = 0;
        box.high = 255;
        box.shape = { Config::getValue<int64_t>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_HEIGHT" }),
                      Config::getValue<int64_t>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_WIDTH" }),
                      3 };
        box.dtype = DataType::UInteger8;
        observation_space["visual_observation_" + pass] = std::move(box);
    }

    return observation_space;
}

std::map<std::string, Box> CameraAgentController::getStepInfoSpace() const
{
    std::map<std::string, Box> step_info_space;
    Box box;

    box.low = std::numeric_limits<float>::lowest();
    box.high = std::numeric_limits<float>::max();
    box.shape = { -1, 3 };
    box.dtype = DataType::Float32;
    step_info_space["random_points"] = std::move(box);

    return step_info_space;
}

void CameraAgentController::applyAction(const std::map<std::string, std::vector<float>>& action)
{
    const FVector agent_location{ action.at("set_pose").at(0), action.at("set_pose").at(1), action.at("set_pose").at(2) };
    const FRotator agent_rotation{ action.at("set_pose").at(3), action.at("set_pose").at(4), action.at("set_pose").at(5) };

    constexpr bool sweep = false;
    constexpr FHitResult* hit_result_info = nullptr;

    camera_actor_->SetActorLocationAndRotation(agent_location, agent_rotation, sweep, hit_result_info, ETeleportType::TeleportPhysics);

    // store action because we are using it in getStepInfo(...)
    action_ = action;
}

std::map<std::string, std::vector<uint8_t>> CameraAgentController::getObservation() const
{
    std::map<std::string, std::vector<uint8_t>> observation;

    // get render data
    std::map<std::string, TArray<FColor>> render_data = camera_sensor_->getRenderData();

    for (const auto& data : render_data) {
        std::vector<uint8_t> image(Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_HEIGHT" }) *
                                   Config::getValue<unsigned long>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "CAMERA_PARAMETERS", "IMAGE_WIDTH" })  * 3);

        for (uint32 i = 0u; i < static_cast<uint32>(data.second.Num()); ++i) {
            image.at(3 * i + 0) = data.second[i].R;
            image.at(3 * i + 1) = data.second[i].G;
            image.at(3 * i + 2) = data.second[i].B;
        }

        observation["visual_observation_" + data.first] = std::move(image);
    }

    return observation;
}

std::map<std::string, std::vector<uint8_t>> CameraAgentController::getStepInfo() const
{
    ASSERT(action_.count("set_num_random_points") && action_.at("set_num_random_points").size() == 1);

    std::map<std::string, std::vector<uint8_t>> step_info;
    std::vector<float> random_points;

    uint32_t num_random_points = static_cast<uint32_t>(action_.at("set_num_random_points").at(0));

    for (uint32_t i = 0u; i < num_random_points; ++i) {
        FVector random_position = nav_mesh_->GetRandomPoint().Location;
        random_points.emplace_back(random_position.X);
        random_points.emplace_back(random_position.Y);
        random_points.emplace_back(random_position.Z);
    }

    step_info["random_points"] = Serialize::toUint8(random_points);

    return step_info;
}

void CameraAgentController::reset()
{}

bool CameraAgentController::isReady() const
{
    return true;
}

void CameraAgentController::buildNavMesh(UNavigationSystemV1* nav_sys)
{
    ASSERT(nav_mesh_);
    ASSERT(nav_sys);

    // Set the NavMesh properties:
    nav_mesh_->CellSize           = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "CELL_SIZE" });
    nav_mesh_->CellHeight         = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "CELL_HEIGHT" });
    nav_mesh_->MergeRegionSize    = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "MERGE_REGION_SIZE" });
    nav_mesh_->MinRegionArea      = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "MIN_REGION_AREA" });
    nav_mesh_->AgentMaxStepHeight = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "AGENT_MAX_STEP_HEIGHT" });
    nav_mesh_->AgentMaxSlope      = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "AGENT_MAX_SLOPE" });
    nav_mesh_->TileSizeUU         = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "TILE_SIZE_UU" });
    nav_mesh_->AgentRadius        = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "AGENT_RADIUS" });
    nav_mesh_->AgentHeight        = Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "AGENT_HEIGHT" });

    // get world bounding box
    FBox world_box(ForceInit);
    std::vector<std::string> world_bound_tag_names = Config::getValue<std::vector<std::string>>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "WORLD_BOUND_TAG_NAMES" });
    for (TActorIterator<AActor> actor_itr(world_); actor_itr; ++actor_itr) {

        for (std::string& tag_itr : world_bound_tag_names) {

            if (actor_itr->ActorHasTag(tag_itr.c_str())) {
                world_box += actor_itr->GetComponentsBoundingBox(false, true);
            }
        }
    }

    // get references to NavMeshBoundsVolume and NavModifierVolume
    ANavMeshBoundsVolume* nav_mesh_bounds_volume = nullptr;
    for (TActorIterator<ANavMeshBoundsVolume> it(world_); it; ++it) {
        nav_mesh_bounds_volume = *it;
    }
    ASSERT(nav_mesh_bounds_volume);

    ANavModifierVolume* nav_modifier_volume = nullptr;
    for (TActorIterator<ANavModifierVolume> it(world_); it; ++it) {
        nav_modifier_volume = *it;
    }
    ASSERT(nav_modifier_volume);

    // update NavMeshBoundsVolume
    nav_mesh_bounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    nav_mesh_bounds_volume->SetActorLocation(world_box.GetCenter(), false);
    nav_mesh_bounds_volume->SetActorRelativeScale3D(world_box.GetSize() / 200.0f);
    nav_mesh_bounds_volume->GetRootComponent()->UpdateBounds();
    nav_sys->OnNavigationBoundsUpdated(nav_mesh_bounds_volume);
    nav_mesh_bounds_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);

    // update NavModifierVolume
    nav_modifier_volume->GetRootComponent()->SetMobility(EComponentMobility::Movable);
    nav_modifier_volume->SetActorLocation(world_box.GetCenter(), false);
    nav_modifier_volume->SetActorRelativeScale3D(world_box.GetSize() / 200.f);
    nav_modifier_volume->AddActorWorldOffset(FVector(Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "NAV_MODIFIER_OFFSET_X" }),
                                                     Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "NAV_MODIFIER_OFFSET_Y" }),
                                                     Config::getValue<float>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "NAV_MODIFIER_OFFSET_Z" })));
    nav_modifier_volume->GetRootComponent()->UpdateBounds();
    nav_modifier_volume->GetRootComponent()->SetMobility(EComponentMobility::Static);
    nav_modifier_volume->RebuildNavigationData();

    nav_sys->Build(); // Rebuild NavMesh, required for update AgentRadius

    // We need to wrap this call with guards because ExportNavigationData(...) is only implemented in non-shipping builds, see:
    //     Engine/Source/Runtime/Engine/Public/AI/NavDataGenerator.h
    //     Engine/Source/Runtime/NavigationSystem/Public/NavMesh/RecastNavMeshGenerator.h
    //     Engine/Source/Runtime/NavigationSystem/Private/NavMesh/RecastNavMeshGenerator.cpp
#if !(UE_BUILD_SHIPPING || UE_BUILD_TEST)
    if (Config::getValue<bool>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "EXPORT_NAV_DATA_OBJ" })) {
        nav_mesh_->GetGenerator()->ExportNavigationData(FString(Config::getValue<std::string>({ "SIMULATION_CONTROLLER", "CAMERA_AGENT_CONTROLLER", "NAVMESH", "EXPORT_NAV_DATA_OBJ_DIR" }).c_str()) + "/" + world_->GetName() + "/");
    }
#endif

}