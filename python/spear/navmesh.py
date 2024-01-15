#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import numpy as np
import spear


class NavMesh():
    def __init__(self, config, simulation_controller):

        self.__config = config
        self._simulation_controller = simulation_controller

    def get_random_points(self, num_points):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        points = self._get_random_points(num_points)
        self._simulation_controller.end_tick()

        return points

    def get_random_reachable_points_in_radius(self, reference_points, radius):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        points = self._get_random_reachable_points_in_radius(reference_points, radius)
        self._simulation_controller.end_tick()

        return points

    def get_paths(self, initial_points, goal_points):

        self._simulation_controller.begin_tick()
        self._simulation_controller.tick()
        points = self._get_paths(initial_points, goal_points)
        self._simulation_controller.end_tick()

        return points
    
    def _get_random_points(self, num_points):
        random_points = self._simulation_controller.rpc_client.call("navmesh.get_random_points", num_points)
        return np.asarray(random_points, dtype=np.float64).reshape(num_points, 3)

    def _get_random_reachable_points_in_radius(self, reference_points, search_radius):
        assert reference_points.shape[1] == 3
        reachable_points = self._simulation_controller.rpc_client.call("navmesh.get_random_reachable_points_in_radius", reference_points.flatten().tolist(), search_radius)
        return np.asarray(reachable_points, dtype=np.float64).reshape(reference_points.shape)

    def _get_paths(self, initial_points, goal_points):
        assert initial_points.shape[1] == 3
        assert goal_points.shape[1] == 3
        paths = self._simulation_controller.rpc_client.call("navmesh.get_paths", initial_points.flatten().tolist(), goal_points.flatten().tolist())
        return [ np.asarray(path, dtype=np.float64).reshape(-1, 3) for path in paths ]
