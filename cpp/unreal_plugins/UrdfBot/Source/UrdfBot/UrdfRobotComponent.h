//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>

#include <CoreMinimal.h>
#include <Components/SceneComponent.h>

#include "UrdfLinkComponent.h"

#include "UrdfRobotComponent.generated.h"

class UUrdfJointComponent;
struct UrdfJointDesc;
struct UrdfRobotDesc;

// unreal representation for urdf robot
UCLASS()
class UUrdfRobotComponent : public USceneComponent
{
    GENERATED_BODY()
public:
    UUrdfRobotComponent(const FObjectInitializer& object_initializer);

    UUrdfLinkComponent* root_link_component_;
    std::map<std::string, UUrdfLinkComponent*> link_components_;
    std::map<std::string, UUrdfJointComponent*> joint_components_;

private:
    // recursively attach child link to parent link
    void attachChildLinks(UrdfJointDesc* joint_desc, const UrdfRobotDesc& robot_desc, UUrdfLinkComponent* parent_link, UUrdfLinkComponent* child_link);
};
