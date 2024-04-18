//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#include "SpEngine/CppFuncService.h"

#include <Engine/Engine.h> // GEngine
#include <Engine/World.h>  // FWorldDelegates

#include "SpCore/Assert.h"
#include "SpCore/Log.h"

void CppFuncService::postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    if (world->IsGameWorld() && GEngine->GetWorldContextFromWorld(world)) {
        SP_ASSERT(!world_);
        world_ = world;
    }
}

void CppFuncService::worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources)
{
    SP_LOG_CURRENT_FUNCTION();
    SP_ASSERT(world);

    if (world == world_) {
        world_ = nullptr;
    }
}

UCppFuncComponent* CppFuncService::getCppFuncComponent(UObject* object)
{
    UCppFuncComponent* cpp_func_component = nullptr;
    if (object->IsA(AActor::StaticClass())) {
        AActor* actor = const_cast<AActor*>(static_cast<const AActor*>(object));
        bool include_all_descendants = false;
        cpp_func_component = Unreal::getChildComponentByType<AActor, UCppFuncComponent>(actor, include_all_descendants);
    } else if (object->IsA(USceneComponent::StaticClass())) {
        USceneComponent* component = const_cast<USceneComponent*>(static_cast<const USceneComponent*>(object));
        bool include_all_descendants = false;
        cpp_func_component = Unreal::getChildComponentByType<USceneComponent, UCppFuncComponent>(component, include_all_descendants);
    } else {
        SP_ASSERT(false);
    }
    SP_ASSERT(cpp_func_component);
    return cpp_func_component;
}
