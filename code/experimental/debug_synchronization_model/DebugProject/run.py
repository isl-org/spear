"""
This is an example code to debug sychronization model used in interiorsim and SimulationController plugin.
"""

import argparse
import cv2
import numpy as np

from interiorsim import config
from interiorsim import Env

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--config_files", nargs="*")

    args = parser.parse_args()

    # create a list of config files to be used
    config_files = []

    # add config file from input if valid
    if args.config_files:
        for file in args.config_files:
            config_files.append(file)

    # load configs
    config = config.get_config(config_files)

    env = Env(config)

    print("python: pinging...")

    print("python: printing action space...")
    print(env.action_space)

    print("python: printing observation space...")
    print(env.observation_space)

    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    # For this section, you'll have to modify SimulationController.cpp with the following changes to observe the effects:
    # in the constructor of SimulationController(), change default frame_state from idle to request_pre_tick
    # in the beginning of beginTick() function, comment ASSERT() statement
    # at the end of endFrameEventHandler() function, change frame_state.store() from idle to request_pre_tick
    # print("#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#")
    # # IMPORTANT!
    # # here, the image captured before tick() should be blank because scenecapturecomponent2D which is responsible for image capture will be paused when game starts. 
    # # unless you set the camera attached to scenecapturecomponent2D to tick when paused, the image will be blank

    # env._beginTick()

    # env._tick()

    # img_1 = env._getObservation()["camera_1_image"] # will be blank image

    # env._endTick()
    # img_2 = env._getObservation()["camera_1_image"] # will be blank image

    # cv2.imshow("1st end frame image (should be blank)", cv2.cvtColor(img_1.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)
    # cv2.imshow("2nd begin frame image (should be blank)", cv2.cvtColor(img_2.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)

    # # ---------------------------- #
    # env._beginTick()

    # env._tick()

    # img_3 = env._getObservation()["camera_1_image"]  # will not be blank image

    # env._endTick()
    # img_4 = env._getObservation()["camera_1_image"]  # will not be blank image

    # cv2.imshow("2nd end frame image (should not be blank)", cv2.cvtColor(img_3.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)
    # cv2.imshow("3rd begin frame image (should not be blank)", cv2.cvtColor(img_4.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)

    # # ---------------------------- #
    # env._beginTick()

    # env._tick()

    # img_5 = env._getObservation()["camera_1_image"]  # will not be blank image

    # env._endTick()
    # img_6 = env._getObservation()["camera_1_image"]  # will not be blank image
 
    # cv2.imshow("3rd end frame image (should not be blank)", cv2.cvtColor(img_5.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)
    # cv2.imshow("4th begin frame image (should not be blank)", cv2.cvtColor(img_6.astype(np.uint8), cv2.COLOR_BGR2RGB))
    # cv2.waitKey(0)

    # cv2.destroyAllWindows()
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    print("#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#")
    
    env.reset()

    #-----------------------------------after applying action, checking actor location just before tick() and after tick()----------------------------------------------------#
    env._beginTick()

    action = {"set_location": [0, 0, 20]}
    env._applyAction(action)
    print(f"python: applying action set_location...{[0, 0, 20]}")
    
    # arr = env._getObservation()["location"] # will raise error if uncommented. To execute this, remove assert() statement in getObservation() function on rpc server
    # print(f"python: printing location before tick() {arr}")

    env._tick()

    arr = env._getObservation()["location"]
    print(f"python: printing location after tick() {arr}")

    env._endTick()

    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    print("#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#")   
    
    print("======================================python: applying apply_force actions======================================")    
    for i in range(5):
        print("-------------------------------------------------------------------------------------------------")
        env._beginTick()

        env._applyAction({"apply_force": [300000]})
        print(f"python: applying action of force multiplier value {300000}")

        # arr = env._getObservation()["location"] # will raise error if uncommented. To execute this, remove assert() statement in getObservation() function on rpc server
        # print(f"python: before tick actor location....{arr}")

        # arr_b1 = env._getObservation()["camera_1_image"] # will raise error if uncommented. To execute this, remove assert() statement in getObservation() function on rpc server
        # arr_b2 = env._getObservation()["camera_1_image"] # will raise error if uncommented. To execute this, remove assert() statement in getObservation() function on rpc server
        # assert np.all(arr_b1 == arr_b2)

        # if i>=1:
            # assert np.all(arr_b2 == arr_a2)

        env._tick()

        arr_a1 = env._getObservation()["camera_1_image"]
        arr_a2 = env._getObservation()["camera_1_image"]
        assert np.all(arr_a1 == arr_a2)

        # assert not np.allclose(arr_a2, arr_b2)

        arr = env._getObservation()["location"]
        print(f"python: after tick action location.... {arr}")
        
        # cv2.imshow("before tick image observation", cv2.cvtColor(arr_b2.astype(np.uint8), cv2.COLOR_BGR2RGB))
        # cv2.waitKey(0)

        cv2.imshow("end tick image observation", cv2.cvtColor(arr_a2.astype(np.uint8), cv2.COLOR_BGR2RGB))
        cv2.waitKey(0)

        env._endTick()

    cv2.destroyAllWindows()
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#

    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#
    # print("#-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#")

    print("======================================python: applying set_location actions======================================")
    for i in range(5):
        print("-------------------------------------------------------------------------------------------------")
        
        print(f"python: applying action...{i*10, 0, 15}")
        obs, reward, done, _ = env.step({"set_location": [i*10, 0, 15]})

        print(f"python: after tick actor location.... {obs['location']}")
    #-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------#

    # close your unreal executable environment gracefully
    env.close()
