#include "VWPhysicsManager.h"

#include <iostream>

#include "PhysicalMaterials/PhysicalMaterial.h"

bool VWPhysicsManager::updatePhysicalMaterial(std::vector<AActor*>& actors, int physical_material_id)
{
        std::cout << "VWPhysicsManager::updatePhysicalMaterial " <<std::endl;
    FString physical_material_id_str = FString::FromInt(physical_material_id);
    FString physical_material_name = FString::Printf(TEXT("/Game/Scene/PhyMaterials/PM_%s.PM_%s"), *physical_material_id_str, *physical_material_id_str);
    UPhysicalMaterial* override_physical_material = LoadObject<UPhysicalMaterial>(nullptr, *physical_material_name);
    if (override_physical_material == nullptr) {
        std::cout << "override_physical_material not found "<< physical_material_id <<std::endl;
        return false;
    }
    for (auto& actor : actors) {
        TArray<UStaticMeshComponent*> components;
        actor->GetComponents<UStaticMeshComponent>(components);
        for (auto& component : components) {
            TArray<UMaterialInterface*> materials;
            component->GetUsedMaterials(materials);
            for (auto& material : materials){
                if (!material->IsA(UMaterialInstanceDynamic::StaticClass())) {
                    UMaterialInstanceDynamic* dynamic_material = UMaterialInstanceDynamic::Create(material, component, FName(material->GetName() + "_Dynamic"));
                    dynamic_material->PhysMaterial = override_physical_material;
                    // update material
                    component->SetMaterial(0, dynamic_material);
                }
                else {
                    UMaterialInstanceDynamic* dynamic_material = Cast<UMaterialInstanceDynamic>(material);
                    dynamic_material->PhysMaterial = override_physical_material;
                    //update physical material if material is not changed
                    FBodyInstance* body_instance = component->GetBodyInstance();
                    if (body_instance && body_instance->IsValidBodyInstance()) {
                        body_instance->UpdatePhysicalMaterials();
                    }
                }
            }
        }
    }
    return true;
}
