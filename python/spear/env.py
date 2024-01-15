#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

from enum import Enum
import gym.spaces
import mmap
import multiprocessing.shared_memory
import numpy as np
import spear
import sys


class Env(gym.Env):
    def __init__(self, config, simulation_controller):

        super(Env, self).__init__()

        self._config = config
        self._simulation_controller = simulation_controller

        self._byte_order = self._simulation_controller.get_byte_order()

        self._action_space_desc = SpaceDesc(self._get_action_space(), dict_space_type=gym.spaces.Dict, box_space_type=gym.spaces.Box)
        self._observation_space_desc = SpaceDesc(self._get_observation_space(), dict_space_type=gym.spaces.Dict, box_space_type=gym.spaces.Box)
        self._task_step_info_space_desc = SpaceDesc(self._get_task_step_info_space(), dict_space_type=Dict, box_space_type=Box)
        self._agent_step_info_space_desc = SpaceDesc(self._get_agent_step_info_space(), dict_space_type=Dict, box_space_type=Box)

        self.action_space = self._action_space_desc.space
        self.observation_space = self._observation_space_desc.space

        self._ready = False

    def step(self, action):
        
        self._simulation_controller.begin_tick()
        self._apply_action(action)
        self._simulation_controller.tick()
        obs = self._get_observation()
        reward = self._get_reward()
        is_done = not self._ready or self._is_episode_done() # if the last call to reset() failed or the episode is done
        step_info = self._get_step_info()
        self._simulation_controller.end_tick()

        return obs, reward, is_done, step_info

    def reset(self, reset_info=None):
        
        for i in range(self._config.SPEAR.MAX_NUM_TICKS_AFTER_RESET):
            self._simulation_controller.begin_tick()
            if i == 0:
                self._reset() # only reset the simulation once
            self._simulation_controller.tick()
            ready = self._is_ready()
            if ready or i == self._config.SPEAR.MAX_NUM_TICKS_AFTER_RESET - 1:
                obs = self._get_observation() # only get the observation if ready, or if we're about to give up
            self._simulation_controller.end_tick()
            if ready:
                break

        self._ready = ready # store if our reset() attempt was successful or not, so step(...) can return done=True if we were unsuccessful

        if reset_info is not None:
            assert isinstance(reset_info, dict)
            reset_info["success"] = ready

        return obs

    # needed to comply with the gym.Env interface
    def render(self):
        pass

    def close(self):

        self._action_space_desc.terminate()
        self._observation_space_desc.terminate()
        self._task_step_info_space_desc.terminate()
        self._agent_step_info_space_desc.terminate()

    def _get_action_space(self):
        array_desc = self._simulation_controller.rpc_client.call("get_action_space")
        assert len(array_desc) > 0
        return array_desc

    def _get_observation_space(self):
        array_desc = self._simulation_controller.rpc_client.call("get_observation_space")
        assert len(array_desc) > 0
        return array_desc

    def _get_task_step_info_space(self):
        return self._simulation_controller.rpc_client.call("get_task_step_info_space")

    def _get_agent_step_info_space(self):
        return self._simulation_controller.rpc_client.call("get_agent_step_info_space")

    def _apply_action(self, action):

        assert action.keys() == self._action_space_desc.space.spaces.keys()

        action_shared = { name:component for name, component in action.items() if name in self._action_space_desc.space_shared.spaces.keys() }
        self._action_space_desc.set_shared_memory_data(action_shared)

        action_non_shared = { name:component for name, component in action.items() if name in self._action_space_desc.space_non_shared.spaces.keys() }
        action_non_shared_serialized = _serialize_arrays(
            action_non_shared, space=self._action_space_desc.space_non_shared, byte_order=self._byte_order)
        self._simulation_controller.rpc_client.call("apply_action", action_non_shared_serialized)

    def _get_observation(self):

        observation_shared = self._observation_space_desc.shared_memory_arrays

        observation_non_shared_serialized = self._simulation_controller.rpc_client.call("get_observation")
        observation_non_shared = _deserialize_arrays(
            observation_non_shared_serialized, space=self._observation_space_desc.space_non_shared, byte_order=self._byte_order)

        assert len(set(observation_shared.keys()) & set(observation_non_shared.keys())) == 0

        return {**observation_shared, **observation_non_shared}

    def _get_reward(self):
        return self._simulation_controller.rpc_client.call("get_reward")
    
    def _is_episode_done(self):
        return self._simulation_controller.rpc_client.call("is_episode_done")

    def _get_step_info(self):

        task_step_info_shared = self._task_step_info_space_desc.shared_memory_arrays
        agent_step_info_shared = self._agent_step_info_space_desc.shared_memory_arrays

        task_step_info_non_shared_serialized = self._simulation_controller.rpc_client.call("get_task_step_info")
        agent_step_info_non_shared_serialized = self._simulation_controller.rpc_client.call("get_agent_step_info")

        task_step_info_non_shared = _deserialize_arrays(
            task_step_info_non_shared_serialized, space=self._task_step_info_space_desc.space_non_shared, byte_order=self._byte_order)
        agent_step_info_non_shared = _deserialize_arrays(
            agent_step_info_non_shared_serialized, space=self._agent_step_info_space_desc.space_non_shared, byte_order=self._byte_order)

        assert len(set(task_step_info_shared.keys()) & set(task_step_info_non_shared.keys())) == 0
        assert len(set(agent_step_info_shared.keys()) & set(agent_step_info_non_shared.keys())) == 0

        return {
            "task_step_info": {**task_step_info_shared, **task_step_info_non_shared},
            "agent_step_info": {**agent_step_info_shared, **agent_step_info_non_shared}}

    def _reset(self):
        # reset the task first in case it needs to set the pose of actors,
        # then reset agent so it can refine the pose of actors
        self._simulation_controller.rpc_client.call("reset_task")
        self._simulation_controller.rpc_client.call("reset_agent")

    def _is_ready(self):
        return self._simulation_controller.rpc_client.call("is_task_ready") and self._simulation_controller.rpc_client.call("is_agent_ready")


# metadata for describing a space including the shared memory objects
class SpaceDesc():
    def __init__(self, array_descs, dict_space_type, box_space_type):

        # array_descs
        self.array_descs = array_descs
        self.array_descs_shared = { name:array_desc for name, array_desc in self.array_descs.items() if array_desc["use_shared_memory_"] }
        self.array_descs_non_shared = { name:array_desc for name, array_desc in self.array_descs.items() if not array_desc["use_shared_memory_"] }

        # spaces
        self.space = _create_dict_space(self.array_descs, dict_space_type=dict_space_type, box_space_type=box_space_type)
        self.space_shared = _create_dict_space(self.array_descs_shared, dict_space_type=dict_space_type, box_space_type=box_space_type)
        self.space_non_shared = _create_dict_space(self.array_descs_non_shared, dict_space_type=dict_space_type, box_space_type=box_space_type)

        # shared memory
        self.shared_memory_objects = {}
        self.shared_memory_arrays = {}
        for name, array_desc in self.array_descs_shared.items():
            if sys.platform == "win32":
                self.shared_memory_objects[name] = mmap.mmap(
                    -1, np.prod(array_desc["shape_"]) * DATATYPE_TO_DTYPE[array_desc["datatype_"]].itemsize, array_desc["shared_memory_name_"])
                self.shared_memory_arrays[name] = np.ndarray(
                    shape=tuple(array_desc["shape_"]), dtype=DATATYPE_TO_DTYPE[array_desc["datatype_"]], buffer=self.shared_memory_objects[name])
            elif sys.platform in ["darwin", "linux"]:
                self.shared_memory_objects[name] = multiprocessing.shared_memory.SharedMemory(name=array_desc["shared_memory_name_"])
                self.shared_memory_arrays[name] = np.ndarray(
                    shape=tuple(array_desc["shape_"]), dtype=DATATYPE_TO_DTYPE[array_desc["datatype_"]], buffer=self.shared_memory_objects[name].buf)
            else:
                assert False

    def terminate(self):
        self.shared_memory_arrays = {}
        for name, shared_memory_object in self.shared_memory_objects.items():
            if sys.platform == "win32":
                shared_memory_object.close()
            elif sys.platform in ["darwin", "linux"]:
                shared_memory_object.close()
                shared_memory_object.unlink()
            else:
                assert False

    def set_shared_memory_data(self, data):
        assert data.keys() == self.space_shared.spaces.keys()
        for name, component in data.items():
            assert isinstance(component, np.ndarray)
            assert component.shape == self.space_shared.spaces[name].shape
            assert component.dtype == self.space_shared.spaces[name].dtype
            assert (component >= self.space_shared.spaces[name].low).all()
            assert (component <= self.space_shared.spaces[name].high).all()
            self.shared_memory_arrays[name][:] = data[name][:]


# mimics the behavior of gym.spaces.Box but allows shape to have the entry -1
class Box():
    def __init__(self, low, high, shape, dtype):
        self.low = low
        self.high = high
        self.shape = shape
        self.dtype = dtype


# mimics the behavior of gym.spaces.Dict
class Dict():
    def __init__(self, spaces):
        self.spaces = spaces


# enum values must match cpp/unreal_plugins/CoreUtils/ArrayDesc.h
class DataType(Enum):
    Invalid    = -1
    UInteger8  = 0
    Integer8   = 1
    UInteger16 = 2
    Integer16  = 3
    UInteger32 = 4
    Integer32  = 5
    Float16    = 6
    Float32    = 7
    Float64    = 8

DATATYPE_TO_DTYPE = {
    DataType.UInteger8.value:  np.dtype("u1"),
    DataType.Integer8.value:   np.dtype("i1"),
    DataType.UInteger16.value: np.dtype("u2"),
    DataType.Integer16.value:  np.dtype("i2"),
    DataType.UInteger32.value: np.dtype("u4"),
    DataType.Integer32.value:  np.dtype("i4"),
    DataType.Float16.value:    np.dtype("f2"),
    DataType.Float32.value:    np.dtype("f4"),
    DataType.Float64.value:    np.dtype("f8"),
}


# functions for creating Python spaces from C++ array_descs
def _create_dict_space(array_descs, dict_space_type, box_space_type):
    return dict_space_type({ name: _create_box_space(array_desc, box_space_type=box_space_type) for name, array_desc in array_descs.items() })

def _create_box_space(array_desc, box_space_type):
    low = array_desc["low_"]
    high = array_desc["high_"]
    shape = tuple(array_desc["shape_"])
    dtype = DATATYPE_TO_DTYPE[array_desc["datatype_"]]
    return box_space_type(low, high, shape, dtype)


# functions for converting arrays between Python and C++ data
def _deserialize_arrays(data, space, byte_order):
    assert data.keys() == space.spaces.keys()
    return { name: _deserialize_array(component, space=space.spaces[name], byte_order=byte_order) for name, component in data.items() }

def _serialize_arrays(arrays, space, byte_order):
    assert arrays.keys() == space.spaces.keys()
    return { name: _serialize_array(component, space=space.spaces[name], byte_order=byte_order) for name, component in arrays.items() }

def _deserialize_array(data, space, byte_order):
    dtype = space.dtype if byte_order is None else space.dtype.newbyteorder(byte_order)
    array = np.frombuffer(data, dtype=dtype, count=-1).reshape(space.shape)
    assert (array >= space.low).all()
    assert (array <= space.high).all()
    return array

def _serialize_array(array, space, byte_order):
    assert isinstance(array, np.ndarray)
    assert array.shape == space.shape
    assert array.dtype == space.dtype
    assert (array >= space.low).all()
    assert (array <= space.high).all()
    data = array.data if byte_order is None else array.newbyteorder(byte_order).data
    return data
