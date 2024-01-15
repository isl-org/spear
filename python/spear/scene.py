#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np
import spear


class Scene():
    def __init__(self, config, simulation_controller):

        self.__config = config
        self._simulation_controller = simulation_controller

        self.__byte_order = self._simulation_controller.get_byte_order()

    def get_all_actor_names(self):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        actor_names = self._get_all_actor_names()
        self._simulation_controller.end_tick()

        return actor_names
    
    def get_all_scene_component_names(self):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        scene_component_names = self._get_all_scene_component_names()
        self._simulation_controller.end_tick()

        return scene_component_names

    def get_all_actor_locations(self):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        all_actor_locations = self._get_all_actor_locations()
        self._simulation_controller.end_tick()

        return all_actor_locations

    def get_all_actor_rotations(self):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        all_actor_rotations = self._get_all_actor_rotations()
        self._simulation_controller.end_tick()

        return all_actor_rotations

    def get_all_component_world_locations(self):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        all_component_locations = self._get_all_component_world_locations()
        self._simulation_controller.end_tick()

        return all_component_locations

    def get_all_component_world_rotations(self):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        all_component_rotations = self._get_all_component_world_rotations()
        self._simulation_controller.end_tick()

        return all_component_rotations

    def get_actor_locations(self, actor_names):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        actor_locations = self._get_actor_locations(actor_names)
        self._simulation_controller.end_tick()

        return actor_locations

    def get_actor_rotations(self, actor_names):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        actor_rotations = self._get_actor_rotations(actor_names)
        self._simulation_controller.end_tick()

        return actor_rotations

    def get_component_world_locations(self, component_names):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        component_locations = self._get_component_world_locations(component_names)
        self._simulation_controller.end_tick()

        return component_locations

    def get_component_world_rotations(self, component_names):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        component_rotations = self._get_component_world_rotations(component_names)
        self._simulation_controller.end_tick()

        return component_rotations

    def get_static_mesh_components_for_actors(self, actor_names):
        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        static_mesh_components = self._get_static_mesh_components_for_actors(actor_names)
        self._simulation_controller.end_tick()

        return static_mesh_components

    def get_physics_constraint_components_for_actors(self, actor_names):
        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        physics_constraint_components = self._get_physics_constraint_components_for_actors(actor_names)
        self._simulation_controller.end_tick()

        return physics_constraint_components

    def is_using_absolute_location(self, component_names):
        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        absolute_locations = self._is_using_absolute_location(component_names)
        self._simulation_controller.end_tick()

        return absolute_locations

    def is_using_absolute_rotation(self, component_names):
        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        absolute_rotations = self._is_using_absolute_rotation(component_names)
        self._simulation_controller.end_tick()

        return absolute_rotations

    def is_using_absolute_scale(self, component_names):
        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        absolute_scales = self._is_using_absolute_scale(component_names)
        self._simulation_controller.end_tick()

        return absolute_scales

    def set_absolute(self, component_names, locations, rotations, scales):
        self._simulation_controller.begin_tick()
        self._set_absolute(component_names, locations, rotations, scales)
        self._simulation_controller.tick()
        self._simulation_controller.end_tick()

    def set_actor_locations(self, actor_locations):

        self._simulation_controller.begin_tick()
        self._set_actor_locations(actor_locations)
        self._simulation_controller.tick()
        self._simulation_controller.end_tick()

    def set_actor_rotations(self, actor_rotations):

        self._simulation_controller.begin_tick()
        self._set_actor_rotations(actor_rotations)        
        self._simulation_controller.tick()
        self._simulation_controller.end_tick()

    def set_component_world_locations(self, component_locations):
        self._simulation_controller.begin_tick()
        self._set_component_world_locations(component_locations)
        self._simulation_controller.tick()
        self._simulation_controller.end_tick()

    def set_component_world_rotations(self, component_rotations):
        self._simulation_controller.begin_tick()
        self._set_component_world_rotations(component_rotations)
        self._simulation_controller.tick()
        self._simulation_controller.end_tick()

    def set_component_relative_locations(self, component_locations):
        self._simulation_controller.begin_tick()
        self._set_component_relative_locations(component_locations)
        self._simulation_controller.tick()
        self._simulation_controller.end_tick()

    def set_component_relative_rotations(self, component_rotations):
        self._simulation_controller.begin_tick()
        self._set_component_relative_rotations(component_rotations)
        self._simulation_controller.tick()
        self._simulation_controller.end_tick()

    def _get_all_actor_names(self):
        return self._simulation_controller.rpc_client.call("scene.get_all_actor_names")

    def _get_all_scene_component_names(self):
        return self._simulation_controller.rpc_client.call("scene.get_all_scene_component_names")

    def _get_all_actor_locations(self):
        all_actor_locations = self._simulation_controller.rpc_client.call("scene.get_all_actor_locations")
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return {name: np.frombuffer(actor_location, dtype=dtype, count=-1) for name, actor_location in all_actor_locations.items()}

    def _get_all_actor_rotations(self):
        all_actor_rotations = self._simulation_controller.rpc_client.call("scene.get_all_actor_rotations")
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return {name: np.frombuffer(actor_rotation, dtype=dtype, count=-1) for name, actor_rotation in all_actor_rotations.items()}

    def _get_all_component_world_locations(self):
        all_component_world_locations = self._simulation_controller.rpc_client.call("scene.get_all_component_world_locations")
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return {name: np.frombuffer(actor_location, dtype=dtype, count=-1) for name, actor_location in all_component_world_locations.items()}

    def _get_all_component_world_rotations(self):
        all_component_world_rotations = self._simulation_controller.rpc_client.call("scene.get_all_component_world_rotations")
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return {name: np.frombuffer(actor_rotation, dtype=dtype, count=-1) for name, actor_rotation in all_component_world_rotations.items()}

    def _get_actor_locations(self, actor_names):
        actor_locations = self._simulation_controller.rpc_client.call("scene.get_actor_locations", actor_names)
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return np.frombuffer(actor_locations, dtype=dtype, count=-1).reshape(-1, 3)

    def _get_actor_rotations(self, actor_names):
        actor_rotations = self._simulation_controller.rpc_client.call("scene.get_actor_rotations", actor_names)
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return np.frombuffer(actor_rotations, dtype=dtype, count=-1).reshape(-1, 4)

    def _get_component_world_locations(self, component_names):
        component_world_locations = self._simulation_controller.rpc_client.call("scene.get_component_world_locations", component_names)
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return np.frombuffer(component_world_locations, dtype=dtype, count=-1).reshape(-1, 3)

    def _get_component_world_rotations(self, component_names):
        component_world_rotations = self._simulation_controller.rpc_client.call("scene.get_component_world_rotations", component_names)
        dtype = np.dtype("f8") if self.__byte_order is None else np.dtype("f8").newbyteorder(self.__byte_order)
        return np.frombuffer(component_world_rotations, dtype=dtype, count=-1).reshape(-1, 4)

    def _get_static_mesh_components_for_actors(self, actor_names):
        return self._simulation_controller.rpc_client.call("scene.get_static_mesh_components_for_actors", actor_names)

    def _get_physics_constraint_components_for_actors(self, actor_names):
        return self._simulation_controller.rpc_client.call("scene.get_physics_constraint_components_for_actors", actor_names)

    def _is_using_absolute_location(self, component_names):
        return self._simulation_controller.rpc_client.call("scene.is_using_absolute_location", component_names)

    def _is_using_absolute_rotation(self, component_names):
        return self._simulation_controller.rpc_client.call("scene.is_using_absolute_rotation", component_names)

    def _is_using_absolute_scale(self, component_names):
        return self._simulation_controller.rpc_client.call("scene.is_using_absolute_scale", component_names)

    def _set_absolute(self, component_names, locations, rotations, scales):
        self._simulation_controller.rpc_client.call("scene.set_absolute", component_names, locations, rotations, scales)

    def _set_actor_locations(self, actor_locations):
        actor_locations_ = {}
        for name, actor_location in actor_locations.items():
            assert isinstance(actor_location, np.ndarray)
            assert actor_location.dtype == np.dtype("f8")
            assert actor_location.shape[0] == 3
            actor_locations_[name] = actor_location.data if self.__byte_order is None else actor_location.newbyteorder(self.__byte_order).data
        self._simulation_controller.rpc_client.call("scene.set_actor_locations", actor_locations_)

    def _set_actor_rotations(self, actor_rotations):
        actor_rotations_ = {}
        for name, actor_location in actor_rotations.items():
            assert isinstance(actor_location, np.ndarray)
            assert actor_location.dtype == np.dtype("f8")
            assert actor_location.shape[0] == 4
            actor_rotations_[name] = actor_location.data if self.__byte_order is None else actor_location.newbyteorder(self.__byte_order).data
        self._simulation_controller.rpc_client.call("scene.set_actor_rotations", actor_rotations_)

    def _set_component_world_locations(self, component_locations):
        component_locations_ = {}
        for name, component_location in component_locations.items():
            assert isinstance(component_location, np.ndarray)
            assert component_location.dtype == np.dtype("f8")
            assert component_location.shape[0] == 3
            component_locations_[name] = component_location.data if self.__byte_order is None else component_location.newbyteorder(self.__byte_order).data
        self._simulation_controller.rpc_client.call("scene.set_component_world_locations", component_locations_)
 
    def _set_component_world_rotations(self, component_rotations):
        component_rotations_ = {}
        for name, component_rotation in component_rotations.items():
            assert isinstance(component_rotation, np.ndarray)
            assert component_rotation.dtype == np.dtype("f8")
            assert component_rotation.shape[0] == 4
            component_rotations_[name] = component_rotation.data if self.__byte_order is None else component_rotation.newbyteorder(self.__byte_order).data
        self._simulation_controller.rpc_client.call("scene.set_component_world_rotations", component_rotations_)

    def _set_component_relative_locations(self, component_locations):
        component_locations_ = {}
        for name, component_location in component_locations.items():
            assert isinstance(component_location, np.ndarray)
            assert component_location.dtype == np.dtype("f8")
            assert component_location.shape[0] == 3
            component_locations_[name] = component_location.data if self.__byte_order is None else component_location.newbyteorder(self.__byte_order).data
        self._simulation_controller.rpc_client.call("scene.set_component_relative_locations", component_locations_)

    def _set_component_relative_rotations(self, component_rotations):
        component_rotations_ = {}
        for name, component_rotation in component_rotations.items():
            assert isinstance(component_rotation, np.ndarray)
            assert component_rotation.dtype == np.dtype("f8")
            assert component_rotation.shape[0] == 4
            component_rotations_[name] = component_rotation.data if self.__byte_order is None else component_rotation.newbyteorder(self.__byte_order).data
        self._simulation_controller.rpc_client.call("scene.set_component_relative_rotations", component_rotations_)
