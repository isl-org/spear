#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import shutil
import spear
import time

# import observation_utils from common folder
COMMON_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), ".."))
import sys
sys.path.append(COMMON_DIR)
import common.observation_utils as observation_utils


# Unreal Engine's rendering system assumes coherence between frames to achieve maximum image quality. 
# However, in this example, we are teleporting the camera in an incoherent way. Hence, we implement a 
# CustomEnv that can render multiple internal frames per step(), so that Unreal Engine's rendering
# system is warmed up by the time we get observations. Doing this can improve overall image quality
# due to Unreal's strategy of accumulating rendering information across multiple frames. These extra
# frames are not necessary in typical embodied AI scenarios, but are useful when teleporting a camera.
class CustomEnv(spear.Env):

    def __init__(self, config, instance, num_internal_steps):
        super(CustomEnv, self).__init__(config, instance)
        assert num_internal_steps > 0
        self._num_internal_steps = num_internal_steps

    def step(self, action):

        if self._num_internal_steps == 1:
            return self.single_step(action, get_observation=True)
        else:
            self.single_step(action)
            for _ in range(1, self._num_internal_steps - 1):
                self.single_step()
            return self.single_step(get_observation=True)

    def single_step(self, action=None, get_observation=False):
    
        spear.begin_tick(self._instance)
        if action:
            self._apply_action(action)
        spear.tick(self._instance)
        if get_observation:
            obs = self._get_observation()
            reward = self._get_reward()
            is_done = self._is_episode_done()
            step_info = self._get_step_info()
            spear.end_tick(self._instance)
            return obs, reward, is_done, step_info
        else:
            spear.end_tick(self._instance)
            return None, None, None, None


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--poses_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "poses.csv")))
    parser.add_argument("--images_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "images")))
    parser.add_argument("--num_internal_steps", type=int, default=10)
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--wait_for_key_press", action="store_true")
    args = parser.parse_args()

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # modify config
    config.defrost()
    config.SPEAR.INSTANCE.COMMAND_LINE_ARGS.renderoffscreen = True
    config.freeze()

    # configure system based on config
    spear.configure_system(config)

    # read data from csv
    df = pd.read_csv(args.poses_file)

    # create SpEngine object
    sp_instance = spear.Instance(config)

    # create Env object
    env = CustomEnv(config, sp_instance, num_internal_steps=args.num_internal_steps)

    # iterate over all poses
    prev_scene_id = ""

    for pose in df.to_records():

        # if the scene_id of our current pose has changed, then create a new Env
        if pose["scene_id"] != prev_scene_id:

            # create dir for storing images
            if not args.benchmark:
                for render_pass in config.SP_ENGINE.LEGACY.CAMERA_AGENT.CAMERA.RENDER_PASSES:
                    render_pass_dir = os.path.realpath(os.path.join(args.images_dir, pose["scene_id"], render_pass))
                    shutil.rmtree(render_pass_dir, ignore_errors=True)
                    os.makedirs(render_pass_dir)

            # close the previous Env
            env.close()

            # open the desired level
            spear.open_level(sp_instance, pose["scene_id"])

            # create Env object
            env = CustomEnv(config, sp_instance, num_internal_steps=args.num_internal_steps)

            # reset the simulation
            _ = env.reset()

            if args.benchmark and prev_scene_id == "":
                start_time_seconds = time.time()

        obs, _, _, _ = env.step(
            action={
                "set_location": np.array([pose["location_x"], pose["location_y"], pose["location_z"]], np.float64),
                "set_rotation": np.array([pose["rotation_pitch"], pose["rotation_yaw"], pose["rotation_roll"]], np.float64)})

        # save images for each render pass
        if not args.benchmark:
            observation_components_to_modify = { render_pass: ["camera." + render_pass] for render_pass in config.SP_ENGINE.LEGACY.CAMERA_AGENT.CAMERA.RENDER_PASSES }
            modified_obs = observation_utils.get_observation_components_modified_for_visualization(obs, observation_components_to_modify)

            for render_pass in config.SP_ENGINE.LEGACY.CAMERA_AGENT.CAMERA.RENDER_PASSES:
                render_pass_dir = os.path.realpath(os.path.join(args.images_dir, pose["scene_id"], render_pass))
                assert os.path.exists(render_pass_dir)

                obs_render_pass_vis = modified_obs["camera." + render_pass]
                plt.imsave(os.path.realpath(os.path.join(render_pass_dir, "%04d.png"%pose["index"])), obs_render_pass_vis)

        # useful for comparing the game window to the image that has been saved to disk
        if args.wait_for_key_press:
            spear.log("Press any key to continue...")
            input()

        prev_scene_id = pose["scene_id"]

    if args.benchmark:
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        spear.log(
            "Average frame time: %0.4f ms (%0.4f fps)" %
            ((elapsed_time_seconds / (df.shape[0]*args.num_internal_steps))*1000, (df.shape[0]*args.num_internal_steps) / elapsed_time_seconds))

    # close the current Env
    env.close()

    # close the unreal instance
    sp_instance.close()

    spear.log("Done.")
