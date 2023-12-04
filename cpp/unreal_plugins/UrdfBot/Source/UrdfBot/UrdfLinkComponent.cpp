//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "UrdfBot/UrdfLinkComponent.h"

#include <iostream>

#include "CoreUtils/Assert.h"
#include "CoreUtils/Config.h"
#include "CoreUtils/Unreal.h"
#include "UrdfBot/UrdfParser.h"

void UUrdfLinkComponent::BeginPlay()
{
    UStaticMeshComponent::BeginPlay();

    // set relative scale at `BeginPlay` to avoid propagating scale to child components
    SetRelativeScale3D(relative_scale_);

    // SetMassOverrideInKg(...) in constructor leads to following warning message during cooking:
    //     Error: FBodyInstance::GetSimplePhysicalMaterial : GEngine not initialized! Cannot call this during
    //     native CDO construction, wrap with if(!HasAnyFlags(RF_ClassDefaultObject)) or move out of constructor,
    //     material parameters will not be correct.
    SetMassOverrideInKg(NAME_None, mass_, true);
}

void UUrdfLinkComponent::initializeComponent(UrdfLinkDesc* link_desc)
{
    // for now link with no visual node or multiple visual node is not supported
    ASSERT(link_desc->visual_descs_.size() == 1);

    UrdfVisualDesc& visual_desc = link_desc->visual_descs_[0];
    UrdfGeometryDesc& geometry_desc = visual_desc.geometry_desc_;

    // set location relative to parent link for non root links
    UrdfJointDesc* joint_desc = link_desc->parent_joint_desc_;
    if (joint_desc) {
        float m_to_cm = 100.0f;
        UrdfLinkDesc* parent_link_desc = joint_desc->parent_link_desc_;
        FTransform link_origin_ = parent_link_desc->visual_descs_[0].origin_;
        SetRelativeLocation((joint_desc->origin_.GetLocation() - link_desc->visual_descs_[0].origin_.GetLocation() - link_origin_.GetLocation()) * m_to_cm);
        SetRelativeRotation(joint_desc->origin_.GetRotation().Rotator() + link_desc->visual_descs_[0].origin_.GetRotation().Rotator());
    }

    UStaticMesh* static_mesh = nullptr;
    switch (geometry_desc.type_) {
        case UrdfGeometryType::Box:
            static_mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube"));
            relative_scale_ = geometry_desc.size_;
            break;
        case UrdfGeometryType::Cylinder:
            static_mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
            relative_scale_ = FVector(geometry_desc.radius_ * 2.0f, geometry_desc.radius_ * 2.0f, geometry_desc.length_);
            break;
        case UrdfGeometryType::Sphere:
            static_mesh = LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
            relative_scale_ = FVector(geometry_desc.radius_) * 2.0f;
            break;
        case UrdfGeometryType::Mesh:
            static_mesh = LoadObject<UStaticMesh>(nullptr, *Unreal::toFString(geometry_desc.filename_));
            relative_scale_ = FVector(geometry_desc.scale_);
            break;
        default:
            ASSERT(false);
    }
    ASSERT(static_mesh);
    SetStaticMesh(static_mesh);

    // set physical property
    SetSimulatePhysics(true);
    SetMobility(EComponentMobility::Movable);
    SetCollisionObjectType(ECollisionChannel::ECC_Vehicle);
    SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
    SetCollisionEnabled(ECollisionEnabled::Type::QueryAndPhysics);
    SetCollisionResponseToChannel(ECollisionChannel::ECC_Vehicle, ECollisionResponse::ECR_Overlap); // ignore collision between robot links
    SetNotifyRigidBodyCollision(true);

    // set never sleep
    GetBodyInstance()->SleepFamily = ESleepFamily::Custom;
    GetBodyInstance()->CustomSleepThresholdMultiplier = 0.f;

    // set physics body's solver iteration count to be better stabalized with more CPU intensive
    GetBodyInstance()->PositionSolverIterationCount = Config::get<float>("URDFBOT.URDF_LINK_COMPONENT.POSITION_SOLVER_ITERATION_COUNT");
    GetBodyInstance()->VelocitySolverIterationCount = Config::get<float>("URDFBOT.URDF_LINK_COMPONENT.VELOCITY_SOLVER_ITERATION_COUNT");

    // set mass
    ASSERT(link_desc->inertial_desc_.mass_ > 0);
    mass_ = link_desc->inertial_desc_.mass_;

    // set rendering material
    if (visual_desc.has_material_) {
        UrdfMaterialDesc* material_desc = &(visual_desc.material_desc_);

        if (material_desc->is_reference_) {
            material_desc = material_desc->material_desc_;
        }
        ASSERT(material_desc);
        if (material_desc->texture_.size() > 0) {
            UMaterialInterface* material = LoadObject<UMaterialInterface>(nullptr, *Unreal::toFString(material_desc->texture_));
            ASSERT(material);

            SetMaterial(0, material);
        } else {
            UMaterialInterface* base_material = LoadObject<UMaterialInterface>(nullptr, TEXT("Material'/UrdfBot/Common/M_PureColor.M_PureColor'"));
            UMaterialInstanceDynamic* material = UMaterialInstanceDynamic::Create(base_material, this);
            material->SetVectorParameterValue("BaseColor_Color", FLinearColor(material_desc->color_));

            SetMaterial(0, material);
        }
    }
}
