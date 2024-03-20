//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <functional> // std::function

#include <Components/ActorComponent.h>
#include <GameFramework/Actor.h>
#include <Math/Vector.h>
#include <UObject/ObjectMacros.h> // GENERATED_BODY, UCLASS, UFUNCTION

#include "SpCore/Log.h"

#include "ActorHitEventComponent.generated.h"

struct FHitResult;

UCLASS()
class UActorHitEventComponent : public UActorComponent
{
    GENERATED_BODY()
public:
    UActorHitEventComponent()
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    ~UActorHitEventComponent()
    {
        SP_LOG_CURRENT_FUNCTION();
    }

    void subscribe(AActor* actor)
    {
        actor->OnActorHit.AddDynamic(this, &UActorHitEventComponent::actorHitHandler);
    }

    void unsubscribe(AActor* actor)
    {
        actor->OnActorHit.RemoveDynamic(this, &UActorHitEventComponent::actorHitHandler);
    }

    void setHandleActorHitFunc(const std::function<void(AActor*, AActor*, FVector, const FHitResult&)>& handle_actor_hit_func)
    {
        handle_actor_hit_func_ = handle_actor_hit_func;
    }

private:
    UFUNCTION()
    void actorHitHandler(AActor* self_actor, AActor* other_actor, FVector normal_impulse, const FHitResult& hit_result)
    {
        if (handle_actor_hit_func_) {
            handle_actor_hit_func_(self_actor, other_actor, normal_impulse, hit_result);
        }
    }

    std::function<void(AActor*, AActor*, FVector, const FHitResult&)> handle_actor_hit_func_;
};
