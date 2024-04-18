#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

class GameWorldService():
    def __init__(self, rpc_client):
        self._rpc_client = rpc_client

    def get_world_name(self):
        return self._rpc_client.call("game_world_service.get_world_name")

    def open_level(self, level_name):
        self._rpc_client.call("game_world_service.open_level", level_name)

    def set_game_paused(self, paused):
        self._rpc_client.call("game_world_service.set_game_paused", paused)

    def find_actors(self):
        return self._rpc_client.call("game_world_service.find_actors")

    def find_actors_as_map(self):
        return self._rpc_client.call("game_world_service.find_actors_as_map")    
    
    def get_components(self, actor):
        return self._rpc_client.call("game_world_service.get_components", actor)

    def get_components_as_map(self, actor):
        return self._rpc_client.call("game_world_service.get_components_as_map", actor)
    
    def get_children_components(self, actor, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components", actor, include_all_descendants)
    
    def get_children_components_as_map(self, actor, include_all_descendants):
        return self._rpc_client.call("game_world_service.get_children_components_as_map", actor, include_all_descendants)
    
    def get_object_properties_as_string(self, object_ptr):
        return self._rpc_client.call("game_world_service.get_object_properties_as_string", object_ptr)

    def get_object_properties_as_string(self, value_ptr, ustruct_ptr):
        return self._rpc_client.call("game_world_service.get_object_properties_as_string", value_ptr, ustruct_ptr)

    def set_object_properties_from_string(self, object_ptr, string):
        self._rpc_client.call("game_world_service.set_object_properties_from_string", object_ptr, string)

    def set_object_properties_from_string(self, value_ptr, ustruct_ptr, string):
        self._rpc_client.call("game_world_service.set_object_properties_from_string", value_ptr, ustruct_ptr, string)
    
    def call_function(self, uobject_ptr, ufunction_ptr):
        return self._rpc_client.call("game_world_service.call_function", uobject_ptr, ufunction_ptr)
    
    def call_function(self, uobject_ptr, ufunction_ptr, args):
        return self._rpc_client.call("game_world_service.call_function", uobject_ptr, ufunction_ptr, args)

    def actor_has_stable_name(self, actor):
        return self._rpc_client.call("game_world_service.actor_has_stable_name", actor)
    
    def component_has_stable_name(self, actor):
        return self._rpc_client.call("game_world_service.component_has_stable_name", actor)
    
    def get_actor_stable_name(self, actor):
        return self._rpc_client.call("game_world_service.get_actor_stable_name", actor)
    
    def get_stable_name_for_actor_component(self, component, include_actor_name):
        return self._rpc_client.call("game_world_service.get_stable_name_for_actor_component", component, include_actor_name)
    
    def get_stable_name_for_scene_component(self, component, include_actor_name):
        return self._rpc_client.call("game_world_service.get_stable_name_for_scene_component", component, include_actor_name)
    
    def set_stable_name(self, actor, stable_name):
        self._rpc_client.call("game_world_service.set_stable_name", actor, stable_name)
    
    def get_actor_tags(self, actor):
        return self._rpc_client.call("game_world_service.get_actor_tags", actor)
    
    def get_component_tags(self, actor):
        return self._rpc_client.call("game_world_service.get_component_tags", actor)
    
    def find_special_struct_by_name(self, name):
        return self._rpc_client.call("game_world_service.find_special_struct_by_name", name)
    
    def request_update_stable_name(self, actor):
        return self._rpc_client.call("game_world_service.request_update_stable_name", actor)
