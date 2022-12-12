# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import pandas as pd
import time
import shutil
import sys

from interiorsim import Env
from interiorsim.config import get_config


class CustomEnv(Env):

    def __init__(self, *args, **kwargs):
        super(CustomEnv, self).__init__(*args, **kwargs)

    def customSetActionTick(self, action):
        
        self._begin_tick()
        self._apply_action(action)
        self._tick()
        self._end_tick()

    def customEmptyTick(self):
        self._begin_tick()
        self._tick()
        self._end_tick()

    def customGetObservationTick(self):
        self._begin_tick()
        self._tick()
        obs = self._get_observation()
        reward = self._get_reward()
        is_done = self._is_episode_done()
        step_info = self._get_step_info()
        self._end_tick()

        return obs, reward, is_done, step_info


if sys.platform == "linux":    
    PLATFORM = "Linux"
elif sys.platform == "darwin":
    PLATFORM = "MacOS"
elif sys.platform == "win32":
    PLATFORM = "Windows"

    
if __name__ == "__main__":
                    
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenes_path", type=str, required=True)
    parser.add_argument("--executable_dir", type=str, required=True)
    parser.add_argument("--poses_file", type=str, required=True)
    parser.add_argument("--output_dir", type=str, required=True)
    args = parser.parse_args()
    
    # read data from csv
    df = pd.read_csv(args.poses_file)

    # load config
    config_files = [ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ]
    config = get_config(config_files)

    NUM_IMAGES_PER_FRAME = 6

    for scene in df["map_id"].unique():

        print()
        print(f"running through scene {scene}...")
        
        # choose map to load
        config.defrost()
        config.INTERIORSIM.MAP_ID = "/Game/Maps/Map_{}".format(scene) # set first scene in the list as starting scene
        config.freeze()

        # copy pak from ssd to disk
        # if not os.path.exists(f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_{PLATFORM}.pak"):
            # shutil.copy(os.path.join(args.scenes_path, f"{scene}_{PLATFORM}.pak"), f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks")
            # shutil.copy(os.path.join(args.scenes_path, f"{scene}/paks/Windows/{scene}/{scene}_{PLATFORM}.pak"), f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks")

        # check if data path for storing images exists
        for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
            if not os.path.exists(os.path.join(args.output_dir, f"Map_{scene}/{render_pass}")):
                os.makedirs(os.path.join(args.output_dir, f"Map_{scene}/{render_pass}"))

        # create Env object
        env = CustomEnv(config)

        # reset the simulation
        _ = env.reset()

        start_time = time.time()

        # iterate over recorded poses
        for pose in df.loc[df["map_id"] == scene].to_records():

            if "final_color" in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
                env.customSetActionTick({"set_pose": np.array([pose["pos_x_cms"], pose["pos_y_cms"], pose["pos_z_cms"], pose["pitch_degs"], pose["yaw_degs"], pose["roll_degs"]], np.float32), "set_num_random_points": np.array([0], np.uint32)})
                for j in range(0, NUM_IMAGES_PER_FRAME - 2):
                    env.customEmptyTick()
                obs, _, _, _ = env.customGetObservationTick()
            elif "segmentation" in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:
                env.step({"set_pose": np.array([pose["pos_x_cms"], pose["pos_y_cms"], pose["pos_z_cms"], pose["pitch_degs"], pose["yaw_degs"], pose["roll_degs"]], np.float32), "set_num_random_points": np.array([0], np.uint32)})
            else:
                assert False, "render pass mode in config file is not supported. Supported types are 'final_color' and 'segmentation'."

            for render_pass in config.SIMULATION_CONTROLLER.CAMERA_AGENT_CONTROLLER.RENDER_PASSES:

                # cv2.imshow(f"visual_observation_{render_pass}", obs[f"visual_observation_{render_pass}"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
                # cv2.waitKey(0)
                output_path = os.path.join(args.output_dir, f"Map_{scene}/{render_pass}")
                assert os.path.exists(output_path)
                return_status = cv2.imwrite(output_path +f"/{pose['index']}.png", obs[f"visual_observation_{render_pass}"][:,:,[2,1,0]]) # OpenCV expects BGR instead of RGB
                assert return_status == True
        
        stop_time = time.time()
        env.close()

        print(f"elapsed time for scene {scene}", stop_time-start_time)
        time.sleep(5)
        # os.remove(f"{args.executable_dir}/{PLATFORM}NoEditor/RobotProject/Content/Paks/{scene}_{PLATFORM}.pak")  
        cv2.destroyAllWindows()
