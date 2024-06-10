#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import json

class UnrealService():
    def __init__(self, rpc_client):
        self._rpc_client = rpc_client

    def get_world_name(self):
        return self._rpc_client.call("unreal_service.get_world_name")

    #
    # Get default object
    #

    def get_default_object(self, uclass, create_if_needed=True):
        return self._rpc_client.call("unreal_service.get_default_object", uclass, create_if_needed)

    #
    # Get class
    #

    def get_class(self, uobject):
        return self._rpc_client.call("unreal_service.get_class", uobject)

    #
    # Find actors unconditionally and return a list or dict
    #

    def find_actors(self):
        return self._rpc_client.call("unreal_service.find_actors")

    def find_actors_as_dict(self):
        return self._rpc_client.call("unreal_service.find_actors_as_map")    

    #
    # Get components unconditionally and return a list or dict
    #

    def get_components(self, actor):
        return self._rpc_client.call("unreal_service.get_components", actor)

    def get_components_as_dict(self, actor):
        return self._rpc_client.call("unreal_service.get_components_as_map", actor)

    #
    # Get children components unconditionally and return a list or dict
    #

    def get_children_components(self, actor, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components", actor, include_all_descendants)

    def get_children_components_as_dict(self, actor, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_as_map", actor, include_all_descendants)

    #
    # Get and set object properties
    #

    def get_object_properties_from_uobject(self, uobject):
        return json.loads(self._rpc_client.call("unreal_service.get_object_properties_as_string_from_uobject", uobject))

    def get_object_properties_from_ustruct(self, value_ptr, ustruct):
        return json.loads(self._rpc_client.call("unreal_service.get_object_properties_as_string_from_ustruct", value_ptr, ustruct))

    def set_object_properties_for_uobject(self, uobject, properties):
        self._rpc_client.call("unreal_service.set_object_properties_from_string_for_uobject", uobject, json.dumps(properties))

    def set_object_properties_for_ustruct(self, value_ptr, ustruct, properties):
        self._rpc_client.call("unreal_service.set_object_properties_from_string_for_ustruct", value_ptr, ustruct, json.dumps(properties))

    #
    # Find properties
    #

    def find_property_by_name_on_uobject(self, uobject, name):
        return self._rpc_client.call("unreal_service.find_property_by_name_on_uobject", uobject, name)

    def find_property_by_name_on_ustruct(self, value_ptr, ustruct, name):
        return self._rpc_client.call("unreal_service.find_property_by_name_on_ustruct", value_ptr, ustruct, name)

    #
    # Get property values
    #

    def get_property_value(self, property_desc):
        return self._rpc_client.call("unreal_service.get_property_value_as_string", property_desc)

    def set_property_value(self, property_desc, property_value):
        return self._rpc_client.call("unreal_service.set_property_value_from_string", property_desc, property_value)

    #
    # Find functions
    #

    def find_function_by_name(self, uclass, name, include_super_flag="IncludeSuper"):
        return self._rpc_client.call("unreal_service.find_function_by_name", uclass, name, {"IncludeSuperFlag": json.dumps({"Enum": include_super_flag})})

    #
    # Interface for calling functions. When using this interface, pointers must be handled specially. For
    # example, suppose you want to call a function that takes a pointer as input and returns a pointer as
    # output. Suppose that you already have a pointer that you would like to pass as input to the function,
    # that you obtained from another UnrealService function, e.g.,
    #
    #     ptr = unreal_service.find_actor_by_name(...)
    #
    # You would invoke your desired function as follows. After executing this code, return_value_ptr will be
    # in the correct form to pass into other functions in unreal_service.
    # 
    #     args = {"Arg": unreal_service.to_ptr(ptr)}
    #     return_values = unreal_service.call_function(uobject, ufunction, args)
    #     return_value_ptr = unreal_service.from_ptr(return_values["ReturnValue"])
    #

    # The Ptr class is for internal use, and does not need to be instantiated directly by users.
    class Ptr:
        def __init__(self, handle):
            self._handle = handle

        def to_string(self):
            return f"{self._handle:#0{10}x}"

    # Convert a pointer obtained from another UnrealService function into a form that can be passed as an
    # argument to unreal_service.call_function(...).
    def to_ptr(self, handle):
        return UnrealService.Ptr(handle)

    # Convert a return value returned by unreal_service.call_function(...) into a form that can be passed to
    # another UnrealService function.
    def from_ptr(self, str):
        return int(str, 0)

    # call an arbitrary function
    def call_function(self, uobject, ufunction, args={}, world_context="WorldContextObject"):

        arg_strings = {}
        for arg_name, arg in args.items():
            if isinstance(arg, UnrealService.Ptr):
                arg_string = arg.to_string()
            else:
                arg_string = json.dumps(arg)
            arg_strings[arg_name] = arg_string

        return_value_strings = self._rpc_client.call("unreal_service.call_function", uobject, ufunction, arg_strings, world_context)
        return_values = { return_value_name: json.loads(return_value_string) for return_value_name, return_value_string in return_value_strings.items() }

        return return_values

    #
    # Find special structs by name
    #

    def find_special_struct_by_name(self, name):
        return self._rpc_client.call("unreal_service.find_special_struct_by_name", name)

    #
    # Stable name helper functions
    #

    def has_stable_name(self, actor):
        return self._rpc_client.call("unreal_service.has_stable_name", actor)

    def get_stable_name_for_actor(self, actor):
        return self._rpc_client.call("unreal_service.get_stable_name_for_actor", actor)

    def get_stable_name_for_component(self, component, include_actor_name=False):
        return self._rpc_client.call("unreal_service.get_stable_name_for_component", component, include_actor_name)

    def get_stable_name_for_scene_component(self, component, include_actor_name=False):
        return self._rpc_client.call("unreal_service.get_stable_name_for_scene_component", component, include_actor_name)

    #
    # Get actor and component tags
    #

    def get_actor_tags(self, actor):
        return self._rpc_client.call("unreal_service.get_actor_tags", actor)

    def get_component_tags(self, actor):
        return self._rpc_client.call("unreal_service.get_component_tags", actor)

    #
    # Spawn actor
    #

    def spawn_actor(self, class_name, location={"X": 0.0, "Y": 0.0, "Z": 0.0}, rotation={"Roll": 0.0, "Pitch": 0.0, "Yaw": 0.0}, spawn_parameters=""):
        return self._rpc_client.call("unreal_service.spawn_actor", class_name, {"Location": json.dumps(location), "Rotation": json.dumps(rotation), "SpawnParameters": json.dumps(spawn_parameters)})

    #
    # Destroy actor
    #

    def destroy_actor(self, actor, net_force=False, should_modify_level=True):
        return self._rpc_client.call("unreal_service.destroy_actor", actor, net_force, should_modify_level)

    #
    # Create component
    #

    def create_component_on_actor(self, class_name, owner, name):
        return self._rpc_client.call("unreal_service.create_component_outside_owner_constructor", class_name, owner, name)

    def create_scene_component_on_actor(self, class_name, owner, name):
        return self._rpc_client.call("unreal_service.create_scene_component_outside_owner_constructor_from_actor", class_name, owner, name)

    def create_scene_component_on_component(self, class_name, owner, parent, name):
        return self._rpc_client.call("unreal_service.create_scene_component_outside_owner_constructor_from_object", class_name, owner, parent, name)

    def create_scene_component_on_owning_scene_component(self, class_name, owner, name):
        return self._rpc_client.call("unreal_service.create_scene_component_outside_owner_constructor_from_component", class_name, owner, name)

    #
    # Destroy component
    #

    def destroy_component(self, component, promote_children=False):
        self._rpc_client.call("unreal_service.destroy_component", component, promote_children)

    #
    # Create new object
    #

    def new_object(self, class_name, outer, name="", object_flags=["RF_NoFlags"], template=0, copy_transients_from_class_defaults=False, in_instance_graph=0, external_package=0):
        return self._rpc_client.call("unreal_service.new_object", class_name, outer, name, template, copy_transients_from_class_defaults, in_instance_graph, external_package, {"ObjectFlags": object_flags})

    #
    # Load object
    #

    def load_object(self, class_name, outer, name, filename=0, load_flags=["LOAD_None"], sandbox=0, instancing_context=0):
        return self._rpc_client.call("unreal_service.load_object", class_name, outer, name, filename, sandbox, instancing_context, {"LoadFlags": load_flags})

    #
    # Get static class
    #

    def get_static_class(self, class_name):
        return self._rpc_client.call("unreal_service.get_static_class", class_name)

    #
    # Find actors conditionally and return a list or dict
    #

    def find_actors_by_name(self, class_name, names, return_null_if_not_found=True):
        return self._rpc_client.call("unreal_service.find_actors_by_name", class_name, names, return_null_if_not_found)

    def find_actors_by_tag(self, class_name, tag):
        return self._rpc_client.call("unreal_service.find_actors_by_tag", class_name, tag)

    def find_actors_by_tag_any(self, class_name, tags):
        return self._rpc_client.call("unreal_service.find_actors_by_tag_any", class_name, tags)

    def find_actors_by_tag_all(self, class_name, tags):
        return self._rpc_client.call("unreal_service.find_actors_by_tag_all", class_name, tags)

    def find_actors_by_type(self, class_name):
        return self._rpc_client.call("unreal_service.find_actors_by_type", class_name)

    def find_actors_by_name_as_dict(self, class_name, names, return_null_if_not_found=True):
        return self._rpc_client.call("unreal_service.find_actors_by_name_as_map", class_name, names, return_null_if_not_found)

    def find_actors_by_tag_as_dict(self, class_name, tag):
        return self._rpc_client.call("unreal_service.find_actors_by_tag_as_map", class_name, tag)

    def find_actors_by_tag_any_as_dict(self, class_name, tags):
        return self._rpc_client.call("unreal_service.find_actors_by_tag_any_as_map", class_name, tags)

    def find_actors_by_tag_all_as_dict(self, class_name, tags):
        return self._rpc_client.call("unreal_service.find_actors_by_tag_all_as_map", class_name, tags)

    def find_actors_by_type_as_dict(self, class_name):
        return self._rpc_client.call("unreal_service.find_actors_by_type_as_map", class_name)

    #
    # Find actor conditionally
    #

    def find_actor_by_name(self, class_name, name, assert_if_not_found=True):
        return self._rpc_client.call("unreal_service.find_actor_by_name", class_name, name, assert_if_not_found)

    def find_actor_by_tag(self, class_name, tag, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.find_actor_by_tag", class_name, tag, assert_if_not_found, assert_if_multiple_found)

    def find_actor_by_tag_any(self, class_name, tags, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.find_actor_by_tag_any", class_name, tags, assert_if_not_found, assert_if_multiple_found)

    def find_actor_by_tag_all(self, class_name, tags, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.find_actor_by_tag_all", class_name, tags, assert_if_not_found, assert_if_multiple_found)

    def find_actor_by_type(self, class_name, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.find_actor_by_type", class_name, assert_if_not_found, assert_if_multiple_found)

    #
    # Get components conditionally and return a list or dict
    #

    def get_components_by_name(self, class_name, actor, names, return_null_if_not_found=True):
        return self._rpc_client.call("unreal_service.get_components_by_name", class_name, actor, names, return_null_if_not_found)

    def get_components_by_tag(self, class_name, actor, tag):
        return self._rpc_client.call("unreal_service.get_components_by_tag", class_name, actor, tag)

    def get_components_by_tag_any(self, class_name, actor, tags):
        return self._rpc_client.call("unreal_service.get_components_by_tag_any", class_name, actor, tags)

    def get_components_by_tag_all(self, class_name, actor, tags):
        return self._rpc_client.call("unreal_service.get_components_by_tag_all", class_name, actor, tags)

    def get_components_by_type(self, class_name, actor):
        return self._rpc_client.call("unreal_service.get_components_by_type", class_name, actor)

    def get_components_by_name_as_dict(self, class_name, actor, names, return_null_if_not_found=True):
        return self._rpc_client.call("unreal_service.get_components_by_name_as_map", class_name, actor, names, return_null_if_not_found)

    def get_components_by_tag_as_dict(self, class_name, actor, tag):
        return self._rpc_client.call("unreal_service.get_components_by_tag_as_map", class_name, actor, tag)

    def get_components_by_tag_any_as_dict(self, class_name, actor, tags):
        return self._rpc_client.call("unreal_service.get_components_by_tag_any_as_map", class_name, actor, tags)

    def get_components_by_tag_all_as_dict(self, class_name, actor, tags):
        return self._rpc_client.call("unreal_service.get_components_by_tag_all_as_map", class_name, actor, tags)

    def get_components_by_type_as_dict(self, class_name, actor):
        return self._rpc_client.call("unreal_service.get_components_by_type_as_map", class_name, actor)

    #
    # Get component conditionally
    #

    def get_component_by_name(self, class_name, actor, name, assert_if_not_found=True):
        return self._rpc_client.call("unreal_service.get_component_by_name", class_name, actor, name, assert_if_not_found)

    def get_component_by_tag(self, class_name, actor, tag, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_component_by_tag", class_name, actor, tag, assert_if_not_found, assert_if_multiple_found)

    def get_component_by_tag_any(self, class_name, actor, tags, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_component_by_tag_any", class_name, actor, tags, assert_if_not_found, assert_if_multiple_found)

    def get_component_by_tag_all(self, class_name, actor, tags, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_component_by_tag_all", class_name, actor, tags, assert_if_not_found, assert_if_multiple_found)

    def get_component_by_type(self, class_name, actor, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_component_by_type", class_name, actor, assert_if_not_found, assert_if_multiple_found)

    #
    # Get children components conditionally from an actor and return a list or dict
    #

    def get_children_components_by_name_from_actor(self, class_name,  parent, names, include_all_descendants=True, return_null_if_not_found=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_name_from_actor", class_name,  parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_from_actor(self, class_name, parent, tag, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_from_actor", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_any_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_all_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_from_actor(self, class_name, parent, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_type_from_actor", class_name, parent, include_all_descendants)

    def get_children_components_by_name_as_map_from_actor(self, class_name, parent, names, include_all_descendants=True, return_null_if_not_found=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_name_as_map_from_actor", class_name, parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_as_map_from_actor(self, class_name, parent, tag, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_as_map_from_actor", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_as_map_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_any_as_map_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_as_map_from_actor(self, class_name, parent, tags, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_all_as_map_from_actor", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_as_map_from_actor(self, class_name, parent, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_type_as_map_from_actor", class_name, parent, include_all_descendants)

    #
    # Get child component conditionally from an actor
    #

    def get_child_component_by_name_from_actor(self, class_name, parent, name, include_all_descendants=True, assert_if_not_found=True):
        return self._rpc_client.call("unreal_service.get_child_component_by_name_from_actor", class_name, parent, name, include_all_descendants, assert_if_not_found)

    def get_child_component_by_tag_from_actor(self, class_name, parent, tag, include_all_descendants=True, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_child_component_by_tag_from_actor", class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_any_from_actor(self, class_name, parent, tags, include_all_descendants=True, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_child_component_by_tag_any_from_actor", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_all_from_actor(self, class_name, parent, tags, include_all_descendants=True, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_child_component_by_tag_all_from_actor", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_type_from_actor(self, class_name, parent, include_all_descendants=True, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_child_component_by_type_from_actor", class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    #
    # Get children components conditionally from a scene component and return a list or dict
    #

    def get_children_components_by_name_from_scene_component(self, class_name, parent, names, include_all_descendants=True, return_null_if_not_found=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_name_from_scene_component", class_name, parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_from_scene_component(self, class_name, parent, tag, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_from_scene_component", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_any_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_all_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_from_scene_component(self, class_name, parent, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_type_from_scene_component", class_name, parent, include_all_descendants)

    def get_children_components_by_name_as_map_from_scene_component(self, class_name, parent, names, include_all_descendants=True, return_null_if_not_found=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_name_as_map_from_scene_component", class_name, parent, names, include_all_descendants, return_null_if_not_found)

    def get_children_components_by_tag_as_map_from_scene_component(self, class_name, parent, tag, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_as_map_from_scene_component", class_name, parent, tag, include_all_descendants)

    def get_children_components_by_tag_any_as_map_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_any_as_map_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_tag_all_as_map_from_scene_component(self, class_name, parent, tags, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_tag_all_as_map_from_scene_component", class_name, parent, tags, include_all_descendants)

    def get_children_components_by_type_as_map_from_scene_component(self, class_name, parent, include_all_descendants=True):
        return self._rpc_client.call("unreal_service.get_children_components_by_type_as_map_from_scene_component", class_name, parent, include_all_descendants)

    #
    # Get child component conditionally from a scene component
    #

    def get_child_component_by_name_from_scene_component(self, class_name, parent, name, include_all_descendants=True, assert_if_not_found=True):
        return self._rpc_client.call("unreal_service.get_child_component_by_name_from_scene_component", class_name, parent, name, include_all_descendants, assert_if_not_found)

    def get_child_component_by_tag_from_scene_component(self, class_name, parent, tag, include_all_descendants=True, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_child_component_by_tag_from_scene_component", class_name, parent, tag, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_any_from_scene_component(self, class_name, parent, tags, include_all_descendants=True, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_child_component_by_tag_any_from_scene_component", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_tag_all_from_scene_component(self, class_name, parent, tags, include_all_descendants=True, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_child_component_by_tag_all_from_scene_component", class_name, parent, tags, include_all_descendants, assert_if_not_found, assert_if_multiple_found)

    def get_child_component_by_type_from_scene_component(self, class_name, parent, include_all_descendants=True, assert_if_not_found=True, assert_if_multiple_found=True):
        return self._rpc_client.call("unreal_service.get_child_component_by_type_from_scene_component", class_name, parent, include_all_descendants, assert_if_not_found, assert_if_multiple_found)
