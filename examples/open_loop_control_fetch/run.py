#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import cv2
import numpy as np
import os
import pandas as pd
import shutil
import spear
import time

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.csv")))
    parser.add_argument("--save_images", default=True)
    parser.add_argument("--image_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "images")))
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--scene_id", default="kujiale_0000")
    args = parser.parse_args()

    np.set_printoptions(linewidth=200)

    # load config
    config = spear.get_config(user_config_files=[os.path.realpath(os.path.join(os.path.dirname(__file__), "user_config.yaml"))])

    # get pregenerated actions for fetch urdf agent
    df = pd.read_csv(args.actions_file)

    if args.save_images:
        if os.path.exists(args.image_dir):
            shutil.rmtree(args.image_dir)
        os.makedirs(args.image_dir)

    # change config based on current scene
    config.defrost()
    scene_config_file = ""
    if args.scene_id == "default_map":
        # warehouse_0000 doesn't need a rendering mode when referring to its map
        config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = \
            "/Game/Scenes/default/Maps/" + args.scene_id + "." + args.scene_id
        config.SIMULATION_CONTROLLER.LEVEL_NAME = \
            "/Game/Scenes/default/Maps/" + args.scene_id

        # default_map has scene-specific config values
        scene_config_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "scene_config.default_map.yaml")
    elif args.scene_id == "kujiale_0000":
        config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = \
            "/Game/Scenes/" + args.scene_id + "/Maps/" + args.scene_id + "_bake" + "." + args.scene_id + "_bake"
        config.SIMULATION_CONTROLLER.LEVEL_NAME = \
            "/Game/Scenes/" + args.scene_id + "/Maps/" + args.scene_id + "_bake"

        # kujiale_0000 has scene-specific config values
        scene_config_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "scene_config.kujiale_0000.yaml")

    if os.path.exists(scene_config_file):
        config.merge_from_file(scene_config_file)
        # config.SIMULATION_CONTROLLER.SCENE_ID = args.scene_id
    config.freeze()

    # create Env object
    env = spear.Env(config)

    # reset the simulation to get the first observation
    obs = env.reset()

    if args.benchmark:
        start_time_seconds = time.time()
    else:
        cv2.imshow("camera.final_color", obs["camera.final_color"])  # note that spear.Env returns BGRA by default
        cv2.waitKey(1)

    for i, row in df.iterrows():
        action = {k: np.array([v], dtype=np.float32) for k, v in row.to_dict().items()}
        obs, reward, done, info = env.step(action=action)

        if not args.benchmark:
            cv2.imshow("camera.final_color", obs["camera.final_color"])  # note that spear.Env returns BGRA by default
            cv2.waitKey(1)

        if args.save_images:
            cv2.imwrite(os.path.realpath(os.path.join(args.image_dir, f"{i:04d}.jpg")), obs["camera.final_color"])

        if done:
            env.reset()

    if args.benchmark:
        end_time_seconds = time.time()
        elapsed_time_seconds = end_time_seconds - start_time_seconds
        print("[SPEAR | run.py] Average frame time: %0.4f ms (%0.4f fps)" % ((elapsed_time_seconds / df.shape[0]) * 1000.0, df.shape[0] / elapsed_time_seconds))
    else:
        cv2.destroyAllWindows()

    # close the environment
    env.close()

    print("[SPEAR | run.py] Done.")
