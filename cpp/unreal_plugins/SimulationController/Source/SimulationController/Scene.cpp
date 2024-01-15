//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SimulationController/Scene.h"

#include <map>
#include <stack>
#include <string>
#include <utility>  //std::pair, std::make_pair
#include <vector>

#include <Components/SceneComponent.h>  // USceneComponent
#include <Components/StaticMeshComponent.h> // UStaticMeshComponent
#include <GameFramework/Actor.h>    //AActor
#include <PhysicsEngine/PhysicsConstraintComponent.h>   //UPhysicsConstraintComponent

#include "CoreUtils/Assert.h"
#include "CoreUtils/Unreal.h"


void Scene::findObjectReferences(UWorld* world)
{
    SP_ASSERT(world);
    world_ = world;

    // find all AActors in the world
    actors_name_ref_map_ = Unreal::findActorsByTagAllAsMap(world_, {});
    SP_ASSERT(!actors_name_ref_map_.empty());

    // create a mapping between component names and it's reference.
    // The names should match the names defined in MuJoCo export pipeline
    for (auto& element : actors_name_ref_map_) {

        // some actors do not have a root component, skip them
        USceneComponent* root_component = element.second->GetRootComponent();
        if (!root_component) {
            continue;
        }
        std::string root_name = element.first + "." + Unreal::toStdString(root_component->GetName());

        std::stack<std::pair<USceneComponent*, std::string>> components_stack;
        components_stack.emplace(std::make_pair(root_component, root_name));
        
        while (!components_stack.empty()) {

            std::pair<USceneComponent*, std::string> top = components_stack.top();
            components_stack.pop();

            SP_ASSERT(!Std::containsKey(scene_components_name_ref_map_, top.second)); // There shouldn't be two components with the same name
            scene_components_name_ref_map_[top.second] = top.first;

            TArray<USceneComponent*> scene_components = top.first->GetAttachChildren();
            for (auto& sm_comp : scene_components) {
                std::string name = top.second + "." + Unreal::toStdString(sm_comp->GetName());
                components_stack.emplace(std::make_pair(sm_comp, name));
            }
        }
    }
}

void Scene::cleanUpObjectReferences()
{
    scene_components_name_ref_map_.clear();
    actors_name_ref_map_.clear();
    world_ = nullptr;
}

std::vector<std::string> Scene::getAllActorNames()
{
    std::vector<std::string> actor_names;
    for(auto& element: actors_name_ref_map_) {
        actor_names.emplace_back(element.first);
    }
    return actor_names;
}

std::vector<std::string> Scene::getAllSceneComponentNames()
{
    std::vector<std::string> component_names;
    for (auto& element : scene_components_name_ref_map_) {
        component_names.emplace_back(element.first);
    }
    return component_names;
}

std::map<std::string, std::vector<uint8_t>> Scene::getAllActorLocations()
{
    std::map<std::string, std::vector<uint8_t>> actor_locations;
    for (auto& element : actors_name_ref_map_) {
        FVector location = element.second->GetActorLocation();
        actor_locations[element.first] = Std::reinterpretAs<uint8_t>(std::vector<double>{location.X, location.Y, location.Z});
    }
    return actor_locations;
}

std::map<std::string, std::vector<uint8_t>> Scene::getAllActorRotations()
{
    std::map<std::string, std::vector<uint8_t>> actor_rotations;
    for (auto& element : actors_name_ref_map_) {
        FQuat rotation = element.second->GetActorQuat();
        actor_rotations[element.first] = Std::reinterpretAs<uint8_t>(std::vector<double>{rotation.W, rotation.X, rotation.Y, rotation.Z});
    }
    return actor_rotations;
}

std::map<std::string, std::vector<uint8_t>> Scene::getAllComponentWorldLocations()
{
    std::map<std::string, std::vector<uint8_t>> component_locations;
    for (auto& element : scene_components_name_ref_map_) {
        FVector location = element.second->GetComponentLocation();
        component_locations[element.first] = Std::reinterpretAs<uint8_t>(std::vector<double>{location.X, location.Y, location.Z});
    }
    return component_locations;
}

std::map<std::string, std::vector<uint8_t>> Scene::getAllComponentWorldRotations()
{
    std::map<std::string, std::vector<uint8_t>> component_rotations;
    for (auto& element : scene_components_name_ref_map_) {
        FQuat rotation = element.second->GetComponentQuat();
        component_rotations[element.first] = Std::reinterpretAs<uint8_t>(std::vector<double>{rotation.W, rotation.X, rotation.Y, rotation.Z});
    }
    return component_rotations;
}

std::vector<uint8_t> Scene::getActorLocations(std::vector<std::string> actor_names)
{
    std::vector<uint8_t> actor_locations;
    for(auto& actor_name: actor_names){
        SP_ASSERT(actors_name_ref_map_.count(actor_name));
        FVector location = actors_name_ref_map_.at(actor_name)->GetActorLocation();
        std::vector<uint8_t> actor_location = Std::reinterpretAs<uint8_t>(std::vector<double>{location.X, location.Y, location.Z});
        actor_locations.insert(actor_locations.end(), actor_location.begin(), actor_location.end());
    }
    return actor_locations;
}

std::vector<uint8_t> Scene::getActorRotations(std::vector<std::string> actor_names)
{
    std::vector<uint8_t> actor_rotations;
    for(auto& actor_name: actor_names){
        SP_ASSERT(actors_name_ref_map_.count(actor_name));
        FQuat rotation = actors_name_ref_map_.at(actor_name)->GetActorQuat();
        std::vector<uint8_t> actor_rotation = Std::reinterpretAs<uint8_t>(std::vector<double>{rotation.W, rotation.X, rotation.Y, rotation.Z});
        actor_rotations.insert(actor_rotations.end(), actor_rotation.begin(), actor_rotation.end());
    }
    return actor_rotations;
}

std::vector<uint8_t> Scene::getComponentWorldLocations(std::vector<std::string> component_names)
{
    std::vector<uint8_t> component_locations;
    for (auto& component_name : component_names) {
        SP_ASSERT(scene_components_name_ref_map_.count(component_name));
        FVector location = scene_components_name_ref_map_.at(component_name)->GetComponentLocation();
        std::vector<uint8_t> actor_location = Std::reinterpretAs<uint8_t>(std::vector<double>{location.X, location.Y, location.Z});
        component_locations.insert(component_locations.end(), actor_location.begin(), actor_location.end());
    }
    return component_locations;
}

std::vector<uint8_t> Scene::getComponentWorldRotations(std::vector<std::string> component_names)
{
    std::vector<uint8_t> component_rotations;
    for (auto& component_name : component_names) {
        SP_ASSERT(scene_components_name_ref_map_.count(component_name));
        FQuat rotation = scene_components_name_ref_map_.at(component_name)->GetComponentQuat();
        std::vector<uint8_t> actor_rotation = Std::reinterpretAs<uint8_t>(std::vector<double>{rotation.W, rotation.X, rotation.Y, rotation.Z});
        component_rotations.insert(component_rotations.end(), actor_rotation.begin(), actor_rotation.end());
    }
    return component_rotations;
}

std::map<std::string, std::vector<std::string>> Scene::getStaticMeshComponentsForActors(std::vector<std::string> actors)
{
    std::map<std::string, std::vector<std::string>> static_mesh_components;
    for (auto& element : actors_name_ref_map_) {

        TArray<UStaticMeshComponent*> sm_comps;
        element.second->GetComponents<UStaticMeshComponent*>(sm_comps);
        for (auto& sm_comp : sm_comps) {
            std::string name = Unreal::toStdString(sm_comp->GetName());
            static_mesh_components[element.first].emplace_back(name);
        }
    }
    return static_mesh_components;
}

std::map<std::string, std::vector<std::string>> Scene::getPhysicsConstraintComponentsForActors(std::vector<std::string> actors)
{
    std::map<std::string, std::vector<std::string>> physics_constraint_components;
    for (auto& element : actors_name_ref_map_) {

        TArray<UPhysicsConstraintComponent*> pc_comps;
        element.second->GetComponents<UPhysicsConstraintComponent*>(pc_comps);
        for (auto& pc_comp : pc_comps) {
            std::string name = Unreal::toStdString(pc_comp->GetName());
            physics_constraint_components[element.first].emplace_back(name);
        }
    }
    return physics_constraint_components;
}

std::vector<bool> Scene::isUsingAbsoluteLocation(std::vector<std::string> component_names) {
    
    std::vector<bool> absolute_locations;

    for (auto& component_name : component_names) {
        SP_ASSERT(scene_components_name_ref_map_.count(component_name));
        bool is_using_absolute = scene_components_name_ref_map_.at(component_name)->IsUsingAbsoluteLocation();
        absolute_locations.emplace_back(is_using_absolute);
    }
    return absolute_locations;
}

std::vector<bool> Scene::isUsingAbsoluteRotation(std::vector<std::string> component_names) {
    std::vector<bool> absolute_locations;

    for (auto& component_name : component_names) {
        SP_ASSERT(scene_components_name_ref_map_.count(component_name));
        bool is_using_absolute = scene_components_name_ref_map_.at(component_name)->IsUsingAbsoluteRotation();
        absolute_locations.emplace_back(is_using_absolute);
    }
    return absolute_locations;
}

std::vector<bool> Scene::isUsingAbsoluteScale(std::vector<std::string> component_names) {
    std::vector<bool> absolute_locations;

    for (auto& component_name : component_names) {
        SP_ASSERT(scene_components_name_ref_map_.count(component_name));
        bool is_using_absolute = scene_components_name_ref_map_.at(component_name)->IsUsingAbsoluteScale();
        absolute_locations.emplace_back(is_using_absolute);
    }
    return absolute_locations;
}

void Scene::SetAbolute(std::vector<std::string> component_names, std::vector<bool> blocations, std::vector<bool> brotations, std::vector<bool> bscales) {

    SP_ASSERT(component_names.size() == blocations.size());
    SP_ASSERT(blocations.size() == brotations.size());
    SP_ASSERT(brotations.size() == bscales.size());

    std::map<std::string, std::vector<uint8_t>> world_locations, world_rotations;
    for (int i = 0; i < component_names.size(); i++) {
        const auto &component_name = component_names.at(i);
        SP_ASSERT(scene_components_name_ref_map_.count(component_name));
        std::vector<std::string> this_component_names{component_name};
        if (blocations.at(i)) world_locations.emplace(component_name, this->getComponentWorldLocations(this_component_names));
        if (brotations.at(i)) world_rotations.emplace(component_name, this->getComponentWorldRotations(this_component_names));
        scene_components_name_ref_map_.at(component_name)->SetAbsolute(blocations.at(i), brotations.at(i), bscales.at(i));
    }
    this->setComponentWorldLocations(world_locations);
    this->setComponentWorldRotations(world_rotations);
}

void Scene::setActorLocations(std::map<std::string, std::vector<uint8_t>> actor_locations)
{
    for (auto& actor_location : actor_locations) {
        SP_ASSERT(actors_name_ref_map_.count(actor_location.first));

        std::vector<double> location = Std::reinterpretAs<double>(actor_location.second);
        SP_ASSERT(location.size() % 3 == 0);

        FVector location_fvector = { location.at(0), location.at(1), location.at(2) };
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        bool success = actors_name_ref_map_.at(actor_location.first)->SetActorLocation(location_fvector, sweep, hit_result, ETeleportType::TeleportPhysics);
        SP_ASSERT(success);
    }
}

void Scene::setActorRotations(std::map<std::string, std::vector<uint8_t>> actor_rotations)
{
    for (auto& actor_rotation : actor_rotations) {
        SP_ASSERT(actors_name_ref_map_.count(actor_rotation.first));

        std::vector<double> rotation = Std::reinterpretAs<double>(actor_rotation.second);
        SP_ASSERT(rotation.size() % 4 == 0);

        FQuat rotation_fquat = { rotation.at(1), rotation.at(2), rotation.at(3), rotation.at(0) };
        bool success = actors_name_ref_map_.at(actor_rotation.first)->SetActorRotation(rotation_fquat, ETeleportType::TeleportPhysics);
        SP_ASSERT(success);
    }
}

void Scene::setComponentWorldLocations(std::map<std::string, std::vector<uint8_t>> component_locations)
{
    for (auto& component_location : component_locations) {
        SP_ASSERT(scene_components_name_ref_map_.count(component_location.first));

        std::vector<double> location = Std::reinterpretAs<double>(component_location.second);
        SP_ASSERT(location.size() % 3 == 0);

        FVector location_fvector = { location.at(0), location.at(1), location.at(2) };
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        scene_components_name_ref_map_.at(component_location.first)->SetWorldLocation(location_fvector, sweep, hit_result, ETeleportType::TeleportPhysics);
    }
}

void Scene::setComponentWorldRotations(std::map<std::string, std::vector<uint8_t>> component_rotations)
{
    for (auto& component_rotation : component_rotations) {
        SP_ASSERT(scene_components_name_ref_map_.count(component_rotation.first));

        std::vector<double> rotation = Std::reinterpretAs<double>(component_rotation.second);
        SP_ASSERT(rotation.size() % 4 == 0);

        FQuat rotation_fquat = { rotation.at(1), rotation.at(2), rotation.at(3), rotation.at(0) };
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        scene_components_name_ref_map_.at(component_rotation.first)->SetWorldRotation(rotation_fquat, sweep, hit_result, ETeleportType::TeleportPhysics);
    }
}

void Scene::setComponentRelativeLocations(std::map<std::string, std::vector<uint8_t>> component_locations)
{
    for (auto& component_location : component_locations) {
        SP_ASSERT(scene_components_name_ref_map_.count(component_location.first));

        std::vector<double> location = Std::reinterpretAs<double>(component_location.second);
        SP_ASSERT(location.size() % 3 == 0);

        FVector location_fvector = { location.at(0), location.at(1), location.at(2) };
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        scene_components_name_ref_map_.at(component_location.first)->SetRelativeLocation(location_fvector, sweep, hit_result, ETeleportType::TeleportPhysics);
    }
}

void Scene::setComponentRelativeRotations(std::map<std::string, std::vector<uint8_t>> component_rotations)
{
    for (auto& component_rotation : component_rotations) {
        SP_ASSERT(scene_components_name_ref_map_.count(component_rotation.first));

        std::vector<double> rotation = Std::reinterpretAs<double>(component_rotation.second);
        SP_ASSERT(rotation.size() % 4 == 0);

        FQuat rotation_fquat = { rotation.at(1), rotation.at(2), rotation.at(3), rotation.at(0) };
        bool sweep = false;
        FHitResult* hit_result = nullptr;
        scene_components_name_ref_map_.at(component_rotation.first)->SetRelativeRotation(rotation_fquat, sweep, hit_result, ETeleportType::TeleportPhysics);
    }
}
