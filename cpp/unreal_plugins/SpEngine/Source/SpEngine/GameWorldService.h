//
// Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
//

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <Delegates/IDelegateInstance.h> // FDelegateHandle
#include <Engine/World.h>                // FWorldDelegates
#include <Kismet/GameplayStatics.h>

#include "SpCore/Assert.h"
#include "SpCore/Log.h"
#include "SpCore/Rpclib.h"
#include "SpCore/Unreal.h"
#include "SpCore/UnrealClassRegistrar.h"
#include "SpEngine/EntryPointBinder.h"

// We use MSGPACK macros here to define structs that can be passed into, and returned from, the service entry
// points defined below. There are already similar struct defined in SpCore, but we choose to define separate
// structs here for the following reasons. First, we avoid a dependency on RPCLib in SpCore. Second, the data
// needed at the SpCore level is sometimes slightly different from the data needed at the SpEngine level, e.g.,
// PropertyDesc needs to maintain a FProperty pointer but GameWorldServicePropertyDesc doesn't. Third, some
// data types used in SpCore cannot be sent via RPCLib, e.g., FProperty* in PropertyDesc.

MSGPACK_ADD_ENUM(EIncludeSuperFlag::Type);
MSGPACK_ADD_ENUM(ELoadFlags);
MSGPACK_ADD_ENUM(EObjectFlags);

struct GameWorldServicePropertyDesc
{
    uint64_t property_;
    uint64_t value_ptr_;

    MSGPACK_DEFINE_MAP(property_, value_ptr_);
};

class GameWorldService {
public:
    GameWorldService() = delete;
    GameWorldService(CUnrealEntryPointBinder auto* unreal_entry_point_binder)
    {
        SP_ASSERT(unreal_entry_point_binder);

        post_world_initialization_handle_ = FWorldDelegates::OnPostWorldInitialization.AddRaw(this, &GameWorldService::postWorldInitializationHandler);
        world_cleanup_handle_ = FWorldDelegates::OnWorldCleanup.AddRaw(this, &GameWorldService::worldCleanupHandler);

        using TMapStringToString = std::map<std::string, std::string>;
        using TMapStringToInt = std::map<std::string, uint64_t>;

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_game_paused", [this](const bool& paused) -> void {
            SP_ASSERT(world_);
            UGameplayStatics::SetGamePaused(world_, paused);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "open_level", [this](const std::string& level_name) -> void {
            SP_ASSERT(world_);
            SP_LOG("Opening level: ", level_name);
            UGameplayStatics::OpenLevel(world_, Unreal::toFName(level_name));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_world_name", [this]() -> std::string {
            SP_ASSERT(world_);
            return Unreal::toStdString(world_->GetName());
        });

        // Unreal.h functions

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors", [this]() -> std::vector<uint64_t> {
            return Std::reinterpretAsVectorOf<uint64_t>(Unreal::findActors(world_));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_as_map", [this]() -> TMapStringToInt {
            return getIntMapFrom(Unreal::findActorsAsMap(world_));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components", [this](const uint64_t& ptr) -> std::vector<uint64_t> {
            return Std::reinterpretAsVectorOf<uint64_t>(Unreal::getComponents(reinterpretAs<AActor*>(ptr)));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_as_map", [this](const uint64_t& ptr) -> TMapStringToInt {
            return getIntMapFrom(Unreal::getComponentsAsMap(reinterpretAs<AActor*>(ptr)));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components", [this](const uint64_t& ptr, const bool& include_all_descendants) -> std::vector<uint64_t> {
            return Std::reinterpretAsVectorOf<uint64_t>(Unreal::getChildrenComponents(reinterpretAs<USceneComponent*>(ptr), include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_as_map", [this](const uint64_t& ptr, const bool& include_all_descendants) -> TMapStringToInt {
            return getIntMapFrom(Unreal::getChildrenComponentsAsMap(reinterpretAs<USceneComponent*>(ptr), include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_object_properties_as_string_from_object", [this](const uint64_t& ptr) -> std::string {
            return Unreal::getObjectPropertiesAsString(reinterpretAs<UObject*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_object_properties_as_string_from_struct", [this](const uint64_t& value_ptr, const uint64_t& struct_ptr) -> std::string {
            return Unreal::getObjectPropertiesAsString(reinterpretAs<void*>(value_ptr), reinterpretAs<UStruct*>(struct_ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_object_properties_from_string_for_object", [this](const uint64_t& ptr, const std::string& string) -> void {
            Unreal::setObjectPropertiesFromString(reinterpretAs<UObject*>(ptr), string);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_object_properties_from_string_for_struct", [this](const uint64_t& value_ptr, const uint64_t& struct_ptr, const std::string& string) -> void {
            Unreal::setObjectPropertiesFromString(reinterpretAs<void*>(value_ptr), reinterpretAs<UStruct*>(struct_ptr), string);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_property_by_name_from_object", [this](const uint64_t& ptr, const std::string& name) -> GameWorldServicePropertyDesc {
            Unreal::PropertyDesc property_desc = Unreal::findPropertyByName(reinterpretAs<UObject*>(ptr), name);
            GameWorldServicePropertyDesc game_world_property_desc;
            game_world_property_desc.property_ = reinterpretAs<uint64_t>(property_desc.property_);
            game_world_property_desc.value_ptr_ = reinterpretAs<uint64_t>(property_desc.value_ptr_);
            return game_world_property_desc;
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_property_by_name_from_struct", [this](const uint64_t& value_ptr, const uint64_t& struct_ptr, const std::string& name) -> GameWorldServicePropertyDesc {
            Unreal::PropertyDesc property_desc = Unreal::findPropertyByName(reinterpretAs<void*>(value_ptr), reinterpretAs<UStruct*>(struct_ptr), name);
            GameWorldServicePropertyDesc game_world_property_desc;
            game_world_property_desc.property_ = reinterpretAs<uint64_t>(property_desc.property_);
            game_world_property_desc.value_ptr_ = reinterpretAs<uint64_t>(property_desc.value_ptr_);
            return game_world_property_desc;
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_property_value_as_string", [this](const GameWorldServicePropertyDesc& service_property_desc) -> std::string {
            Unreal::PropertyDesc property_desc;
            property_desc.property_ = reinterpretAs<FProperty*>(service_property_desc.property_);
            property_desc.value_ptr_ = reinterpretAs<void*>(service_property_desc.value_ptr_);
            return Unreal::getPropertyValueAsString(property_desc);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "set_property_value_as_string", [this](const GameWorldServicePropertyDesc& service_property_desc, const std::string& string) -> void {
            Unreal::PropertyDesc property_desc;
            property_desc.property_ = reinterpretAs<FProperty*>(service_property_desc.property_);
            property_desc.value_ptr_ = reinterpretAs<void*>(service_property_desc.value_ptr_);
            Unreal::setPropertyValueFromString(property_desc, string);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_function_by_name", [this](const uint64_t& ptr, const std::string& name, const EIncludeSuperFlag::Type& include_super_flag) -> uint64_t {
            return reinterpretAs<uint64_t>(Unreal::findFunctionByName(reinterpretAs<UClass*>(ptr), name, include_super_flag));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "call_function_without_args", [this](const uint64_t& object_ptr, const uint64_t& function_ptr) -> TMapStringToString {
            return Unreal::callFunction(reinterpretAs<UObject*>(object_ptr), reinterpretAs<UFunction*>(function_ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "call_function_with_args", [this](const uint64_t& object_ptr, const uint64_t& function_ptr, const TMapStringToString& args) -> TMapStringToString {
            return Unreal::callFunction(reinterpretAs<UObject*>(object_ptr), reinterpretAs<UFunction*>(function_ptr), args);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "actor_has_stable_name", [this](const uint64_t& ptr) -> bool {
            return Unreal::hasStableName(reinterpretAs<AActor*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "component_has_stable_name", [this](const uint64_t& ptr) -> bool {
            return Unreal::hasStableName(reinterpretAs<UActorComponent*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_stable_name_for_actor", [this](const uint64_t& ptr) -> std::string {
            return Unreal::getStableName(reinterpretAs<AActor*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_stable_name_for_actor_component", [this](const uint64_t& ptr, const bool& include_actor_name) -> std::string {
            return Unreal::getStableName(reinterpretAs<UActorComponent*>(ptr), include_actor_name);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_stable_name_for_scene_component", [this](const uint64_t& ptr, const bool& include_actor_name) -> std::string {
            return Unreal::getStableName(reinterpretAs<USceneComponent*>(ptr), include_actor_name);
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_actor_tags", [this](const uint64_t& ptr) -> std::vector<std::string> {
            return Unreal::getTags(reinterpretAs<AActor*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_component_tags", [this](const uint64_t& ptr) -> std::vector<std::string> {
            return Unreal::getTags(reinterpretAs<UActorComponent*>(ptr));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_special_struct_by_name", [this](const std::string& name) -> uint64_t {
            return reinterpretAs<uint64_t>(Unreal::findSpecialStructByName(name));
        });

        // UnrealClassRegistrar.h functions

        // TODO: How do we expose FActorSpawnParameters?
        //unreal_entry_point_binder->bindFuncUnreal("game_world_service", "spawn_actor",
        //    [this](const std::string& class_name, const std::vector<double>& location, const std::vector<double>& rotation, const spawn_params) -> uint64_t {
        //        return reinterpretAs<uint64_t>(
        //            UnrealClassRegistrar::spawnActor(
        //                class_name,
        //                world_,
        //                FVector(location.at(0), location.at(1), location.at(2)),
        //                FRotator(rotation.at(0), rotation.at(1), rotation.at(2)),
        //                FActorSpawnParameters()));
        //}); // create a USTRUCT copy of FActorSpawnParameters. Then serialize and deserialize to json to transfer over rpclib.

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "create_component_outside_owner_constructor", 
            [this](const std::string& class_name, const uint64_t& owner, const std::string& name) -> uint64_t {
                return reinterpretAs<uint64_t>(UnrealClassRegistrar::createComponentOutsideOwnerConstructor(class_name, reinterpretAs<AActor*>(owner), name));
         });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "create_scene_component_outside_owner_constructor_from_actor",
            [this](const std::string& class_name, const uint64_t& actor, const std::string& name) -> uint64_t {
                return reinterpretAs<uint64_t>(UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(class_name, reinterpretAs<AActor*>(actor), name));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "create_scene_component_outside_owner_constructor_from_object",
            [this](const std::string& class_name, const uint64_t& owner, const uint64_t& parent, const std::string& name) -> uint64_t {
                return reinterpretAs<uint64_t>(UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(class_name, reinterpretAs<UObject*>(owner), reinterpretAs<USceneComponent*>(parent), name));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "create_scene_component_outside_owner_constructor_from_component",
            [this](const std::string& class_name, const uint64_t& owner, const std::string& name) -> uint64_t {
                return reinterpretAs<uint64_t>(UnrealClassRegistrar::createSceneComponentOutsideOwnerConstructor(class_name, reinterpretAs<USceneComponent*>(owner), name));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "create_new_object",
            [this](
                const std::string& class_name,
                const uint64_t& outer,
                const std::string& name,
                const EObjectFlags& flags,
                const uint64_t& uobject_template,
                const bool& copy_transients_from_class_defaults,
                const uint64_t& in_instance_graph,
                const uint64_t& external_package) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::newObject(
                        class_name,
                        reinterpretAs<UObject*>(outer),
                        Unreal::toFName(name),
                        flags,
                        reinterpretAs<UObject*>(uobject_template),
                        copy_transients_from_class_defaults,
                        reinterpretAs<FObjectInstancingGraph*>(in_instance_graph),
                        reinterpretAs<UPackage*>(external_package)));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "load_object",
            [this](
                const std::string& class_name,
                const uint64_t& outer,
                const std::string& name,
                const std::string& filename,
                const ELoadFlags& load_flags,
                const uint64_t& sandbox,
                const uint64_t& instancing_context) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::loadObject(
                        class_name,
                        reinterpretAs<UObject*>(outer),
                        *Unreal::toFString(name),
                        *Unreal::toFString(filename),
                        load_flags,
                        reinterpretAs<UPackageMap*>(sandbox),
                        reinterpretAs<FLinkerInstancingContext*>(instancing_context)));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_by_name",
            [this](const std::string& class_name, const std::vector<std::string>& names,const bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(
                    UnrealClassRegistrar::findActorsByName(
                        class_name,
                        world_,
                        names,
                        return_null_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_by_tag",
            [this](const std::string& class_name, const std::string& tag) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::findActorsByTag(class_name, world_, tag));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_by_tag_any",
            [this](const std::string& class_name, const std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::findActorsByTagAny(class_name, world_, tags));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_by_tag_all",
            [this](const std::string& class_name, const std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::findActorsByTagAll(class_name, world_, tags));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_by_type", [this](const std::string& class_name) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::findActorsByType(class_name,world_));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_by_name_as_map",
            [this](const std::string& class_name, const std::vector<std::string>& names, const bool& return_null_if_not_found) -> TMapStringToInt {
                return getIntMapFrom(
                    UnrealClassRegistrar::findActorsByNameAsMap(
                        class_name,
                        world_,
                        names,
                        return_null_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_by_tag_as_map",
            [this](const std::string& class_name, const std::string& tag) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::findActorsByTagAsMap(class_name, world_, tag));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_by_tag_any_as_map",
            [this](const std::string& class_name, const std::vector<std::string>& tags) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::findActorsByTagAnyAsMap(class_name, world_, tags));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_by_tag_all_as_map",
            [this](const std::string& class_name, const std::vector<std::string>& tags) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::findActorsByTagAllAsMap(class_name, world_, tags));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actors_by_type_as_map",
            [this](const std::string& class_name) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::findActorsByTypeAsMap(class_name,world_));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actor_by_name",
            [this](const std::string& class_name, const std::string& name,const bool& assert_if_not_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::findActorByName(
                        class_name,
                        world_,
                        name,
                        assert_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actor_by_tag",
            [this](const std::string& class_name, const std::string& tag, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::findActorByTag(
                        class_name,
                        world_,
                        tag,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actor_by_tag_any",
            [this](const std::string& class_name, const std::vector<std::string>& tags, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::findActorByTagAny(
                        class_name,
                        world_,
                        tags,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actor_by_tag_all",
            [this](const std::string& class_name, const std::vector<std::string>& tags, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::findActorByTagAll(
                        class_name,
                        world_,
                        tags,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "find_actor_by_type",
            [this](const std::string& class_name, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::findActorByType(
                        class_name,
                        world_,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        // components
        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_by_name",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& names, const bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(
                    UnrealClassRegistrar::getComponentsByName(
                        class_name,
                        reinterpretAs<AActor*>(actor),
                        names,
                        return_null_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_by_tag",
            [this](const std::string& class_name, const uint64_t& actor, const std::string& tag) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getComponentsByTag(class_name, reinterpretAs<AActor*>(actor), tag));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_by_tag_any",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getComponentsByTagAny(class_name, reinterpretAs<AActor*>(actor), tags));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_by_tag_all",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& tags) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getComponentsByTagAll(class_name, reinterpretAs<AActor*>(actor), tags));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_by_type", [this](const std::string& class_name, const uint64_t& actor) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getComponentsByType(class_name, reinterpretAs<AActor*>(actor)));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_by_name_as_map",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& names, const bool& return_null_if_not_found) -> TMapStringToInt {
                return getIntMapFrom(
                    UnrealClassRegistrar::getComponentsByNameAsMap(
                        class_name,
                        reinterpretAs<AActor*>(actor),
                        names,
                        return_null_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_by_tag_as_map",
            [this](const std::string& class_name, const uint64_t& actor, const std::string& tag) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getComponentsByTagAsMap(class_name, reinterpretAs<AActor*>(actor), tag));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_by_tag_any_as_map",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& tags) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getComponentsByTagAnyAsMap(class_name, reinterpretAs<AActor*>(actor), tags));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_by_tag_all_as_map",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& tags) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getComponentsByTagAllAsMap(class_name, reinterpretAs<AActor*>(actor), tags));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_components_by_type_as_map",
            [this](const std::string& class_name, const uint64_t& actor) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getComponentsByTypeAsMap(class_name, reinterpretAs<AActor*>(actor)));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_component_by_name",
            [this](const std::string& class_name, const uint64_t& actor, const std::string& name,const bool& assert_if_not_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getComponentByName(
                        class_name,
                        reinterpretAs<AActor*>(actor),
                        name,
                        assert_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_component_by_tag",
            [this](const std::string& class_name, const uint64_t& actor, const std::string& tag, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getComponentByTag(
                        class_name,
                        reinterpretAs<AActor*>(actor),
                        tag,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_component_by_tag_any",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& tags, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getComponentByTagAny(
                        class_name,
                        reinterpretAs<AActor*>(actor),
                        tags,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_component_by_tag_all",
            [this](const std::string& class_name, const uint64_t& actor, const std::vector<std::string>& tags, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getComponentByTagAll(
                        class_name,
                        reinterpretAs<AActor*>(actor),
                        tags,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_component_by_type",
            [this](const std::string& class_name, const uint64_t& actor, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getComponentByType(
                        class_name,
                        reinterpretAs<AActor*>(actor),
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        // ChildrenComponents from AActor
        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_name_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& names, const bool& include_all_descendants, const bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(
                    UnrealClassRegistrar::getChildrenComponentsByName(
                        class_name,
                        reinterpretAs<AActor*>(parent),
                        names,
                        include_all_descendants,
                        return_null_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& tag, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTag(class_name, reinterpretAs<AActor*>(parent), tag, include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_any_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTagAny(class_name, reinterpretAs<AActor*>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_all_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTagAll(class_name, reinterpretAs<AActor*>(parent), tags, include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_type_from_actor", [this](const std::string& class_name, const uint64_t& parent, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByType(class_name, reinterpretAs<AActor*>(parent), include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_name_as_map_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& names, const bool& include_all_descendants, const bool& return_null_if_not_found) -> TMapStringToInt {
                return getIntMapFrom(
                    UnrealClassRegistrar::getChildrenComponentsByNameAsMap(
                        class_name,
                        reinterpretAs<AActor*>(parent),
                        names,
                        include_all_descendants,
                        return_null_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_as_map_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& tag, const bool& include_all_descendants) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getChildrenComponentsByTagAsMap(class_name, reinterpretAs<AActor*>(parent), tag, include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_any_as_map_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getChildrenComponentsByTagAnyAsMap(class_name, reinterpretAs<AActor*>(parent), tags, include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_all_as_map_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getChildrenComponentsByTagAllAsMap(class_name, reinterpretAs<AActor*>(parent), tags, include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_type_as_map_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const bool& include_all_descendants) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getChildrenComponentsByTypeAsMap(class_name, reinterpretAs<AActor*>(parent), include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_child_component_by_name_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& name, const bool& include_all_descendants, const bool& assert_if_not_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByName(
                        class_name,
                        reinterpretAs<AActor*>(parent),
                        name,
                        include_all_descendants,
                        assert_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_child_component_by_tag_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& tag, const bool& include_all_descendants, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTag(
                        class_name,
                        reinterpretAs<AActor*>(parent),
                        tag,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_child_component_by_tag_any_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTagAny(
                        class_name,
                        reinterpretAs<AActor*>(parent),
                        tags,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_child_component_by_tag_all_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTagAll(
                        class_name,
                        reinterpretAs<AActor*>(parent),
                        tags,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_child_component_by_type_from_actor",
            [this](const std::string& class_name, const uint64_t& parent, const bool& include_all_descendants, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByType(
                        class_name,
                        reinterpretAs<AActor*>(parent),
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        // get children components from SceneComponent*
        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_name_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& names, const bool& include_all_descendants, const bool& return_null_if_not_found) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(
                    UnrealClassRegistrar::getChildrenComponentsByName(
                        class_name,
                        reinterpretAs<USceneComponent*>(parent),
                        names,
                        include_all_descendants,
                        return_null_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& tag, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTag(class_name, reinterpretAs<USceneComponent*>(parent), tag, include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_any_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTagAny(class_name, reinterpretAs<USceneComponent*>(parent), tags, include_all_descendants));
            });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_all_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByTagAll(class_name, reinterpretAs<USceneComponent*>(parent), tags, include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_type_from_scene_component", [this](const std::string& class_name, const uint64_t& parent, const bool& include_all_descendants) -> std::vector<uint64_t> {
                return Std::reinterpretAsVectorOf<uint64_t>(UnrealClassRegistrar::getChildrenComponentsByType(class_name, reinterpretAs<USceneComponent*>(parent), include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_name_as_map_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& names, const bool& include_all_descendants, const bool& return_null_if_not_found) -> TMapStringToInt {
                return getIntMapFrom(
                    UnrealClassRegistrar::getChildrenComponentsByNameAsMap(
                        class_name,
                        reinterpretAs<USceneComponent*>(parent),
                        names,
                        include_all_descendants,
                        return_null_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_as_map_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& tag, const bool& include_all_descendants) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getChildrenComponentsByTagAsMap(class_name, reinterpretAs<USceneComponent*>(parent), tag, include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_any_as_map_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getChildrenComponentsByTagAnyAsMap(class_name, reinterpretAs<USceneComponent*>(parent), tags, include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_tag_all_as_map_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getChildrenComponentsByTagAllAsMap(class_name, reinterpretAs<USceneComponent*>(parent), tags, include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_children_components_by_type_as_map_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const bool& include_all_descendants) -> TMapStringToInt {
                return getIntMapFrom(UnrealClassRegistrar::getChildrenComponentsByTypeAsMap(class_name, reinterpretAs<USceneComponent*>(parent), include_all_descendants));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_child_component_by_name_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& name, const bool& include_all_descendants, const bool& assert_if_not_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByName(
                        class_name,
                        reinterpretAs<USceneComponent*>(parent),
                        name,
                        include_all_descendants,
                        assert_if_not_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_child_component_by_tag_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::string& tag, const bool& include_all_descendants, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTag(
                        class_name,
                        reinterpretAs<USceneComponent*>(parent),
                        tag,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_child_component_by_tag_any_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTagAny(
                        class_name,
                        reinterpretAs<USceneComponent*>(parent),
                        tags,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_child_component_by_tag_all_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const std::vector<std::string>& tags, const bool& include_all_descendants, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByTagAll(
                        class_name,
                        reinterpretAs<USceneComponent*>(parent),
                        tags,
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });

        unreal_entry_point_binder->bindFuncUnreal("game_world_service", "get_child_component_by_type_from_scene_component",
            [this](const std::string& class_name, const uint64_t& parent, const bool& include_all_descendants, const bool& assert_if_not_found, const bool& assert_if_multiple_found) -> uint64_t {
                return reinterpretAs<uint64_t>(
                    UnrealClassRegistrar::getChildComponentByType(
                        class_name,
                        reinterpretAs<USceneComponent*>(parent),
                        include_all_descendants,
                        assert_if_not_found,
                        assert_if_multiple_found));
        });
    }

    ~GameWorldService()
    {
        FWorldDelegates::OnWorldCleanup.Remove(world_cleanup_handle_);
        FWorldDelegates::OnPostWorldInitialization.Remove(post_world_initialization_handle_);

        world_cleanup_handle_.Reset();
        post_world_initialization_handle_.Reset();
    }

    void postWorldInitializationHandler(UWorld* world, const UWorld::InitializationValues initialization_values);
    void worldCleanupHandler(UWorld* world, bool session_ended, bool cleanup_resources);

private:

    template <typename TKey, typename TSrcValue>
    std::map<TKey, uint64_t> getIntMapFrom(std::map<TKey, TSrcValue>&& input_map)
    {
        return Std::toMap<TKey, uint64_t>(
            input_map | 
            std::views::transform([](auto& pair) { auto& [key, value] = pair; return std::make_pair(key, reinterpret_cast<uint64_t>(value)); }));
    }

    template <typename TReturn, typename TSrc>
    TReturn reinterpretAs(TSrc ptr_addr)
    {
        return reinterpret_cast<TReturn>(ptr_addr);
    }

    FDelegateHandle post_world_initialization_handle_;
    FDelegateHandle world_cleanup_handle_;

    UWorld* world_ = nullptr;
};
