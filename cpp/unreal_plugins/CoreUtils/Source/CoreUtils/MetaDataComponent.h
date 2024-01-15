//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include "Components/ActorComponent.h"

#include "MetaDataComponent.generated.h"

class AActor;
class FDelegateHandle;
class FObjectInitializer;

struct FPropertyChangedEvent;

UCLASS(ClassGroup = (SPEAR), meta = (BlueprintSpawnableComponent), HideCategories = (Activation, Actor, 
	AssetUserData, Collision, Cooking, HLOD, Input, Networking, PhysicsRendering, Replication, Tags))
class COREUTILS_API UMetaDataComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UMetaDataComponent(const FObjectInitializer& object_initializer);
	~UMetaDataComponent();

	UPROPERTY(EditInstanceOnly, BlueprintReadWrite, Category = "SPEAR", DisplayName = "Parent Actor Label Name")
	FString parent_actor_label_name_;

#if WITH_EDITOR
	void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

private:
#if WITH_EDITOR
	void onActorLabelChangedEventHandler(AActor*);
	FDelegateHandle on_actor_label_changed_handle_;
#endif
};
