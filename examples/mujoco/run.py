#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.
# this script should be run with [mjpython](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer), not python.
# mjpython is required for the passive mujoco viewer.

import argparse
import cv2
import mujoco
import mujoco.viewer
import numpy as np
import os
import spear
import time
# from scipy.spatial.transform import Rotation as R


osp = os.path


NUM_STEPS = 100


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--mjcf_path", required=True, help="path to scene MJCF file")
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(osp.join(osp.dirname(__file__), "user_config.yaml"))])

    # create mujoco objects
    mujoco_model = mujoco.MjModel.from_xml_path(osp.expanduser(args.mjcf_path))
    mujoco_data = mujoco.MjData(mujoco_model)

    # perform this step once to load all information
    mujoco.mj_forward(mujoco_model, mujoco_data)

    # get all body ids that needs to be updated in every step
    # these body ids are the first child of all joints
    # first child is only to ensure that in case of free joints wherein there are 6 of the same children, we need only one
    # and in every other joint case, there will only be 1 child
    body_ids = [ mujoco_model.joint(jnt_id).bodyid[0] for jnt_id in range(mujoco_model.njnt) ]
    # body_names = [ mujoco_model.body(body_id).name for body_id in body_ids ]

    actor_names = []
    actor_ids = []
    component_names = []
    component_ids = []
    for body_id in body_ids:
        body = mujoco_model.body(body_id)
        body_name = body.name
        component_names.append(body_name)
        component_ids.append(body_id)
        # if body.parentid == 0:  # if parent id world, then body is an actor (in UE terms)
        #     actor_names.append(body_name)
        #     actor_ids.append(body_id)
        # else:
        #     # if "Kitchen_Sink.Door_01" in body_name or "Cabinet.Door_01" in body_name or "Kitchen_Sink.Drawer_01" in body_name:        # uncomment to just send data to actuators
        #     component_names.append(body_name)
        #     component_ids.append(body_id)

    # get all mujoco bodies and corresponding xpos, xquat
    xpos_dict  = {mujoco_model.body(body_id).name: mujoco_data.body(body_id).xpos  for body_id in body_ids}
    xquat_dict = {mujoco_model.body(body_id).name: mujoco_data.body(body_id).xquat for body_id in body_ids}

    print("xpos dict.......................")
    print(xpos_dict)
    print("xquat dict......................")
    print(xquat_dict)

    # create SimulationController object
    simulation_controller = spear.SimulationController(config)

    # create Scene object
    scene = spear.Scene(config, simulation_controller)

    spear.log()
    spear.log("All actor names:")
    spear_actor_names = scene.get_all_actor_names()
    spear.log("printing all actor names\n", spear_actor_names)

    spear.log()
    spear.log("All scene_component names:")
    scene_component_names = scene.get_all_scene_component_names()
    spear.log("printing all scene_component names\n", scene_component_names)

    spear.log()
    spear.log("All actor body names from mujoco:")
    spear.log(actor_names)

    spear.log()
    spear.log("All component body names from mujoco:")
    spear.log(component_names)

    spear.log()
    spear.log("printing is_using_absolute_location for components")
    absolute_locations = scene.is_using_absolute_location(component_names)
    spear.log(absolute_locations)

    spear.log()
    spear.log("printing is_using_absolute_rotation for components")
    absolute_rotations = scene.is_using_absolute_rotation(component_names)
    spear.log(absolute_rotations)

    spear.log()
    spear.log("printing is_using_absolute_scale for components")
    absolute_scales = scene.is_using_absolute_scale(component_names)
    spear.log(absolute_scales)

    spear.log(len(component_names), len(absolute_locations), len(absolute_rotations), len(absolute_scales))
    scene.set_absolute(component_names, [True for _ in absolute_locations], [True for _ in absolute_rotations], absolute_scales)

    rots = scene.get_component_world_rotations(component_names)
    spear.log()
    spear.log("UE rot:   ", {x:y for x,y in zip(component_names, rots)})

    ##############################################
    ####### mujoco and spear communication #######
    viewer = mujoco.viewer.launch_passive(mujoco_model, mujoco_data)

    act_mids = [np.mean(mujoco_model.actuator(i).ctrlrange) for i in range(mujoco_model.nu)]
    act_mags = [0.5*(mujoco_model.actuator(i).ctrlrange[1]-mujoco_model.actuator(i).ctrlrange[0]) for i in range(mujoco_model.nu)]
    period = 1000

    start = time.time()
    t = 0
    muj_update_steps = 20

    while viewer.is_running() and time.time() - start < 300:
        # send actutations to mujoco
        # mujoco_data.actuator("Cabinet/PhysicsConstraint_door_01_x_revolute_actuator").ctrl[0] = 1 # body/component_name Cabinet.Door_01

        for _ in range(muj_update_steps):
            l = np.sign(np.sin(2*np.pi*t/period))

            for i in range(mujoco_model.nu):
                mujoco_data.ctrl[i] = act_mids[i] + l*act_mags[i]

            mujoco.mj_step(mujoco_model, mujoco_data)
            
            # increament time
            t += 1

        viewer.sync()

        # get updated xpos, xquat
        xpos_dict  = {mujoco_model.body(body_id).name: mujoco_data.body(body_id).xpos  for body_id in component_ids}
        xquat_dict = {mujoco_model.body(body_id).name: mujoco_data.body(body_id).xquat for body_id in component_ids}

        spear.log("mujoco xpos:  ", xpos_dict)
        spear.log()
        spear.log("mujoco xquat: ", xquat_dict)    
        spear.log()

        scene.set_component_world_locations(xpos_dict)
        locs = scene.get_component_world_locations(component_names)
        spear.log("UE loc:       ", {x:y for x,y in zip(component_names, locs)})
        spear.log()
        scene.set_component_world_rotations(xquat_dict)
        rots = scene.get_component_world_rotations(component_names)
        spear.log()
        spear.log("UE rot:   ", {x:y for x,y in zip(component_names, rots)})

    viewer.close()
    simulation_controller.close()
    quit()
    
    static_mesh_components = scene.get_static_mesh_components_for_actors(actor_names)
    spear.log("printing all static mesh components\n", static_mesh_components)

    spear.log()
    spear.log("all actor locations")
    actor_locations = scene.get_all_actor_locations()
    for name, actor_location in actor_locations.items():
        spear.log(name, actor_location)
    
    spear.log()
    spear.log("all actor rotations")
    actor_rotations = scene.get_all_actor_rotations()
    for name, actor_rotation in actor_rotations.items():
        spear.log(name, actor_rotation)

    spear.log()
    value = np.array([1000, 1000, 1000], dtype=np.float64)
    spear.log("setting actor location for ", actor_names[4], "value ", value)
    scene.set_actor_locations({actor_names[4]: value})

    spear.log()
    value = np.array([0, 360, 0], dtype=np.float64)
    spear.log("setting actor rotation for ", actor_names[4], "value ", value)
    scene.set_actor_rotations({actor_names[4]: value})

    spear.log()
    spear.log("getting actor locations for ", actor_names[:10])
    actor_locations = scene.get_actor_locations(actor_names[:10])
    for name, actor_location in zip(actor_names[:10], actor_locations):
        spear.log(name, actor_location)

    spear.log()
    spear.log("getting actor rotations for ", actor_names[:10])
    actor_rotations = scene.get_actor_rotations(actor_names[:10])
    for name, actor_rotation in zip(actor_names[:10], actor_rotations):
        spear.log(name, actor_rotation)

    # close the environment
    simulation_controller.close()

    spear.log("Done.")
