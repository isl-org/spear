//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <vector>

class AActor;
class USceneComponent;
class UWorld;

class Scene
{
public:    
    Scene() = default;
    ~Scene() = default;

    void findObjectReferences(UWorld* world);
    void cleanUpObjectReferences();;

    std::vector<std::string> getAllActorNames();
    std::vector<std::string> getAllSceneComponentNames();
    std::map<std::string, std::vector<uint8_t>> getAllActorLocations();
    std::map<std::string, std::vector<uint8_t>> getAllActorRotations();
    std::map<std::string, std::vector<uint8_t>> getAllComponentWorldLocations();
    std::map<std::string, std::vector<uint8_t>> getAllComponentWorldRotations();
    std::vector<uint8_t> getActorLocations(std::vector<std::string> actor_names);
    std::vector<uint8_t> getActorRotations(std::vector<std::string> actor_names);
    std::vector<uint8_t> getComponentWorldLocations(std::vector<std::string> component_names);
    std::vector<uint8_t> getComponentWorldRotations(std::vector<std::string> component_names);
    std::map<std::string, std::vector<std::string>> getStaticMeshComponentsForActors(std::vector<std::string> actors);
    std::map<std::string, std::vector<std::string>> getPhysicsConstraintComponentsForActors(std::vector<std::string> actors);

    std::vector<bool> isUsingAbsoluteLocation(std::vector<std::string> component_names);
    std::vector<bool> isUsingAbsoluteRotation(std::vector<std::string> component_names);
    std::vector<bool> isUsingAbsoluteScale(std::vector<std::string> component_names);

    void SetAbolute(std::vector<std::string> component_names, std::vector<bool> blocations, std::vector<bool> brotations, std::vector<bool> bscales);
    void setActorLocations(std::map<std::string, std::vector<uint8_t>> actor_locations);
    void setActorRotations(std::map<std::string, std::vector<uint8_t>> actor_rotations);
    void setComponentWorldLocations(std::map<std::string, std::vector<uint8_t>> component_locations);
    void setComponentWorldRotations(std::map<std::string, std::vector<uint8_t>> component_rotations);
    void setComponentRelativeLocations(std::map<std::string, std::vector<uint8_t>> component_locations);
    void setComponentRelativeRotations(std::map<std::string, std::vector<uint8_t>> component_rotations);
private:
    UWorld* world_ = nullptr;
    std::map<std::string, AActor*> actors_name_ref_map_;
    std::map<std::string, USceneComponent*> scene_components_name_ref_map_;
};
