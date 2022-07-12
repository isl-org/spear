#include "DoorManager.h"

#include <iostream>

#include <EngineUtils.h>
#include <PhysicsEngine/PhysicsConstraintComponent.h>

#include "Assert.h"

TArray<FDoorInfo> DoorManager::level_door_info_;

bool DoorManager::initLevelDoorInfo(UWorld* world)
{
    ASSERT(world);
    FString map_name = world->GetName();
    // check if map name follows InteriorSim scene
    ASSERT(map_name.StartsWith("Map_"));

    UDataTable* door_data_table = LoadObject<UDataTable>(nullptr, TEXT("DataTable'/SceneManager/Koolab/SceneInfo/DT_scene_door_info.DT_scene_door_info'"));
    FSceneDoorInfo* level_door_info_table = door_data_table->FindRow<FSceneDoorInfo>(FName(map_name.Mid(4)), "doors");
    // check if level_door_info_table is available
    ASSERT(level_door_info_table);
    //cleanup previous data
    level_door_info_.Empty();
    //load door info
    level_door_info_.Append(level_door_info_table->doors);
    //find corresponding door actor in current world
    matchDoorActor(world);
    return true;
}

void DoorManager::matchDoorActor(UWorld* world)
{
    for (TActorIterator<AActor> it(world); it; ++it) {
        // TODO use other way to identify door actors
        if (it->GetName().StartsWith("Group_")) {
            for (auto& child_component : it->GetComponents()) {
                // mark if child name is door component (INSTid1216)
                if (child_component->GetName().StartsWith("Architecture_") && child_component->GetName().Contains("INSTid1216")) {
                    it->Tags.Add("door");
                    FBox door_actor_bbox = it->GetComponentsBoundingBox(false, true);
                    for (auto& door_info : level_door_info_) {
                        if (door_info.doorActor != nullptr){
                            //skip registered door_info
                            continue;
                        }
                        // check if its position is in door bbox
                        FVector closest_point_to_expected = door_actor_bbox.GetClosestPointTo(door_info.position);
                        if (door_info.position.Equals(closest_point_to_expected)) {
                            //position from door info is inside actor
                            door_info.doorActor = *it;
                            break;
                        }
                    }
                    //only need to match first child component as door
                    break;
                }
            }
        }
    }
    // check all door_info find its actor. Warn in case if changed by other scene manipulation
    for (auto& door_info : level_door_info_) {
        if (door_info.doorActor == nullptr) {
            std::cout << "DoorManager::matchDoorActor - door not matched at" << TCHAR_TO_UTF8(*door_info.position.ToString()) << std::endl;
        }
    }
}

bool DoorManager::moveAllDoor(bool open)
{
    for (auto& door_info : level_door_info_) {
        AActor* door_actor = door_info.doorActor;
        // skip door_actor if unmatched in case if changed by other scene manipulation.
        if (!door_actor) {
            continue;
        }
        if (door_info.mode == "counter-clockwise" || door_info.mode == "clockwise") {
            TArray<UStaticMeshComponent*> child_components;
            door_actor->GetComponents(child_components);
            for (auto& child_component : child_components) {
                // find Animation component
                if (child_component->GetName().StartsWith("Animation_")) {
                    UStaticMeshComponent* animation_component = Cast<UStaticMeshComponent>(child_component);
                    FTransform animation_transform = FTransform(animation_component->GetRelativeTransform());
                    //only open inner door. skip if it goes to outside
                    if (door_info.isInnerDoor) {
                        if (open){
                            if (door_info.mode == "counter-clockwise") {
                                animation_transform.SetRotation(FRotator(0, -85, 0).Quaternion());
                            }else if (door_info.mode == "clockwise"){
                                animation_transform.SetRotation(FRotator(0, 85, 0).Quaternion());
                            }
                        }else{
                            animation_transform.SetRotation(FRotator(0, 0, 0).Quaternion());
                        }
                        animation_component->SetRelativeTransform(animation_transform, false, nullptr, ETeleportType::ResetPhysics);
                    }
                }
            }
        }
        else if (door_info.mode == "sliding") {
            // find all animation components for movement
            TArray<UStaticMeshComponent*> animation_components;
            door_actor->GetComponents(animation_components);
            animation_components.RemoveAll([](UStaticMeshComponent* child_component) {
                return !child_component->GetName().StartsWith("Animation_");
            });
            // check if animation_components_size is in [2,3,4]
            int animation_components_size = animation_components.Num();
            if (animation_components_size < 2 || animation_components_size > 4) {
                //unexpected number of sliding door
                ASSERT(false);
            }

            // if there are multiple movable animation component, only move one of them 
            UStaticMeshComponent* farthest_component = nullptr;
            float farthest_distance = 0;
            UStaticMeshComponent* closest_component = nullptr;
            float closest_distance = TNumericLimits<float>::Max();

            for (auto& animation_component : animation_components) {
                // calculate anmi bounding box
                FBox animation_component_bbox = FBox(EForceInit::ForceInit);
                TArray<USceneComponent*> children_scene_components;
                animation_component->GetChildrenComponents(true, children_scene_components);
                for (auto& child_scene_component : children_scene_components) {
                    if (child_scene_component->IsRegistered() && child_scene_component->IsCollisionEnabled()) {
                        animation_component_bbox += child_scene_component->Bounds.GetBox();
                    }
                }
                // find farthest and nearest component as target
                float distance = FVector::Distance(animation_component->GetComponentLocation(), animation_component_bbox.GetCenter());
                if (distance > farthest_distance) {
                    farthest_distance = distance;
                    farthest_component = animation_component;
                }
                if (distance < closest_distance) {
                    closest_distance = distance;
                    closest_component = animation_component;
                }
            }
            // move sliding anmi depending on total Anmi number
            if (animation_components_size == 2 || animation_components_size == 3) {
                // only move max door
                FVector old_location = farthest_component->GetRelativeLocation() * FVector(1.75f, 1, 1);
                farthest_component->SetRelativeLocation(old_location, false, nullptr, ETeleportType::ResetPhysics);
            }
            if (animation_components_size == 4) {
                farthest_component->SetRelativeLocation(farthest_component->GetRelativeLocation() * FVector(1.75f, 1, 1), false, nullptr, ETeleportType::ResetPhysics);
                closest_component->SetRelativeLocation(farthest_component->GetRelativeLocation() * FVector(-0.75f, 1, 1), false, nullptr, ETeleportType::ResetPhysics);
            }
        }
        else {
            // Unknown door_info.mode, should not occur
            ASSERT(false);
        }
    }
    return true;
}
