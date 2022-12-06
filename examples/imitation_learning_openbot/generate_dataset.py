# Before running this file, rename user_config.yaml.example -> user_config.yaml and modify it with appropriate paths for your system.

import argparse
import datetime
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import shutil
import spear
import time

from policies import OpenBotPIDPolicy
from utils import *
  
  
if __name__ == "__main__":

    # parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--iterations", default=500)
    parser.add_argument("--episodes_file", default=os.path.join(os.path.dirname(os.path.realpath(__file__)), "train_episodes.csv"))
    parser.add_argument("--rendering_mode", default="baked")
    parser.add_argument("--create_plot", action="store_true")
    parser.add_argument("--create_video", action="store_true")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--scene_id")
    args = parser.parse_args()
    
    # build the episode data folder and its subfolders following the guidelines of the OpenBot public repository 
    # https://github.com/isl-org/OpenBot/tree/master/policy#data-collection
    base_dir = os.path.dirname(os.path.dirname(__file__))
    dataset_dir = os.path.join(base_dir, "dataset")
    video_dir = os.path.join(dataset_dir, "videos")
    train_data_dir = os.path.join(dataset_dir, "train_data")
    test_data_dir = os.path.join(dataset_dir, "test_data")
    
    # load config
    config = spear.get_config(user_config_files=[ os.path.join(os.path.dirname(os.path.realpath(__file__)), "user_config.yaml") ])

    # make sure that we are not in trajectory sampling mode
    config.defrost()
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.GET_POSITIONS_FROM_TRAJECTORY_SAMPLING = False
    config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.POSITIONS_FILE = args.episodes_file
    config.freeze()

    # string to load a different map depending on the rendering mode
    if args.rendering_mode == "baked":
        rendering_mode_map_str = "_bake"
    elif args.rendering_mode == "raytracing":
        rendering_mode_map_str = "_rtx"
    else:
        assert False

    # handle debug configuration (markers are only produed in Developent configuration; NOT in Shipping configuration)
    if args.debug:
        config.defrost()
        config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_SAMPLING_DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.IMU_SENSOR.DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.SONAR_SENSOR.DEBUG_RENDER = True
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_HEIGHT = 1080
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_WIDTH = 1920
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES = ["final_color", "segmentation", "depth_glsl"]
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS = ["state_data", "control_data", "camera", "imu", "sonar"]
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_X = -50.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_Y = -50.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.POSITION_Z = 45.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.PITCH = -35.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.YAW = 45.0
        config.OPENBOT.OPENBOT_PAWN.CAMERA_COMPONENT.ROLL = 0.0
        config.freeze()
    else:
        config.defrost()
        config.SIMULATION_CONTROLLER.IMITATION_LEARNING_TASK.TRAJECTORY_SAMPLING_DEBUG_RENDER = False
        config.SIMULATION_CONTROLLER.IMU_SENSOR.DEBUG_RENDER = False
        config.SIMULATION_CONTROLLER.SONAR_SENSOR.DEBUG_RENDER = False
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_HEIGHT = 120
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.IMAGE_WIDTH= 160
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES = ["final_color"]
        config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS = ["state_data", "control_data", "camera"]
        config.freeze()

    # load driving policy
    driving_policy = OpenBotPIDPolicy(config)
    
    # sanity checks (without these observation modes, the code will not behave properly)
    assert("state_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("control_data" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("camera" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS)
    assert("final_color" in config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES)
    
    # if the user provides a scene_id, use it, otherwise use the scenes defined in scenes.csv
    if args.scene_id is None:
        scenes_csv_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), "scenes.csv")
        assert os.path.exists(scenes_csv_file)
        scene_ids = pd.read_csv(scenes_csv_file, dtype={"scene_id":str})["scene_id"]
    else:
        scene_ids = [args.scene_id]

    # loop through the desired set of scenes
    for i, scene_id in enumerate(scene_ids):
        
        # change config based on current scene
        config.defrost()
        config.SIMULATION_CONTROLLER.WORLD_PATH_NAME = "/Game/Maps/Map_" + scene_id + rendering_mode_map_str + "." + "Map_" + scene_id + rendering_mode_map_str
        config.SIMULATION_CONTROLLER.LEVEL_NAME = "/Game/Maps/Map_" + scene_id + rendering_mode_map_str
        config.SIMULATION_CONTROLLER.SCENE_ID = scene_id
        config.freeze()

        # load the episodes to be executed
        assert os.path.exists(episodes_csv_file)
        episodes_csv_file = pd.read_csv(args.episodes_file, dtype={"scene_id":str})
        number_of_episodes = len(episodes_csv_file)

        # create Env object
        env = spear.Env(config=config)
        
        episode = 0
        # execute the desired number of episodes in a given scene
        while episode < number_of_episodes:

            exp_dir = f"episode_{episode}_{scene_id}"
            
            # split data between training and evaluation sets
            #if 100 * episode < config.DRIVING_POLICY.PERCENTAGE_OF_TRAINING_DATA * args.runs:
            #    episode_dir = os.path.join(train_data_dir, exp_dir) 
            #else:
            #    episode_dir = os.path.join(test_data_dir, exp_dir) 
                
            data_dir = os.path.join(episode_dir,"data")
            image_dir = os.path.join(data_dir, "images")
            sensor_dir = os.path.join(data_dir, "sensor_data")
            os.makedirs(data_dir, exist_ok=True)
            os.makedirs(image_dir, exist_ok=True)
            os.makedirs(sensor_dir, exist_ok=True)

            print("----------------------")
            print(f"episode {episode} over {XXX}")
            print("----------------------")

            # reset the simulation
            _ = env.reset()
        
            # send zero action to the agent and collect initial trajectory observations:
            obs, _, _, info = env.step({"apply_voltage": np.array([0.0, 0.0], dtype=np.float32)})

            # initialize the driving policy with the desired trajectory 
            driving_policy.set_trajectory(info["agent_step_info"]["trajectory_data"])

            collision_flag = False # flag raised when the vehicle collides with the environment. It restarts the episode without iterating the episode count
            control_data_buffer = np.empty([args.iterations, 2], dtype=np.float32) # buffer containing the control_data observations made by the agent during a episode
            state_data_buffer = np.empty([args.iterations, 6], dtype=np.float32) # buffer containing the state_data observations made by the agent during an episode
            waypoint_data_buffer = np.empty([args.iterations, 3], dtype=np.float32) # buffer containing the waypoint coordinates being tracked by the agent during an episode
            time_data_buffer = np.empty([args.iterations, 1], dtype=np.int32) # buffer containing the time stamps of the observations made by the agent during an episode
            frame_data_buffer = np.empty([args.iterations, 1], dtype=np.int32) # buffer containing the frame ids
            executed_iterations = 0
            
            # execute the desired number of iterations in a given episode
            for i in range(args.iterations):

                print(f"iteration {i} of {args.iterations}")

                time_stamp = int(10000*datetime.datetime.now().timestamp())

                # update control action 
                action, policy_step_info = driving_policy.step(obs)

                # send control action to the agent and collect observations
                obs, _, _, info = env.step({"apply_voltage": action})

                # debug
                if args.debug:
                    show_obs(obs, config.SIMULATION_CONTROLLER.OPENBOT_AGENT.OBSERVATION_COMPONENTS, config.SIMULATION_CONTROLLER.OPENBOT_AGENT.CAMERA.RENDER_PASSES)

                # save the collected rgb observations
                plt.imsave(os.path.join(image_dir, "%d.jpeg"%i), obs["camera_final_color"].squeeze())

                # During an episode, there is no guarantee that the agent reaches the predefined goal although its behavior is perfectly valid for training purposes. 
                # In practice, it may for instance occur that the agent is not given enough time steps or control authority to move along the whole trajectory. 
                # In this case, rather than considering the whole episode as a fail, one can consider the last position reached by the agent as the new goal position. 
                # Doing so requires a recomputation of the compass observation, since the latter is goal dependant. Therefore, rather than directly writing all the 
                # observations in a file iteration by iteration, we append these observations in a buffer, named "observation_buffer" to later process them once the 
                # episode is completed. 
                control_data_buffer[i] = obs["control_data"]    # control_data: [ctrl_left, ctrl_right]
                state_data_buffer[i] = obs["state_data"]        # state_data: [x, y, z, pitch, yaw, roll]
                waypoint_data_buffer[i] = policy_step_info["current_waypoint"]# current waypoint being tracked by the agent
                time_data_buffer[i] = time_stamp                # current time stamp
                frame_data_buffer[i] = i                        # current frame
                executed_iterations = executed_iterations + 1

                if info["task_step_info"]["hit_obstacle"]: # if the vehicle collided with an obstacle
                    print("Collision detected ! Killing simulation and restarting episode...")
                    collision_flag = True # raise collision_flag to interrupt and restart the current episode
                    break
                elif info["task_step_info"]["hit_goal"] or policy_step_info["goal_reached"]: # if the vehicle reached the goal
                    print("Goal reached !")
                    break # interrupt the current episode and move to the next episode
            
            # episode loop executed: check the termination flags
            
            if collision_flag: # if the collision flag is raised during the episode
                print("Restarting episode...")
                shutil.rmtree(episode_dir) # remove the data collected so far as it is improper for training purposes
                # do not update the episode count as the episode must be restarted

            else: # as no collision occured during the episode, the collected data can be used for training purposes

                # populate the observation data files with the observation_buffer buffer content
                print("Filling database...")

                # set the goal position as the last position reached by the agent
                goal_position_xy = np.array([state_data_buffer[executed_iterations-1][0],state_data_buffer[executed_iterations-1][1]], dtype=np.float32) # use the vehicle last x-y location as goal for the episode

                for it in range(executed_iterations):
               
                    # low-level commands sent to the motors
                    df_ctrl = pd.DataFrame({"timestamp[ns]"  : time_data_buffer[it],
                            "left_ctrl" : control_data_buffer[it][0],
                            "right_ctrl" : control_data_buffer[it][1]})
                    df_ctrl.to_csv(os.path.join(sensor_dir,"ctrlLog.txt"), mode="a", index=False, header=it==0)

                
                    # get the updated compass observation (with the last recorded position set as goal)
                    position_xy_current = np.array([state_data_buffer[it][0], state_data_buffer[it][1]], dtype=np.float32)
                    yaw_current = state_data_buffer[it][4]
                    compass_observation = get_compass_observation(goal_position_xy, position_xy_current, yaw_current)

                    # high level commands
                    df_goal = pd.DataFrame({"timestamp[ns]"  : time_data_buffer[it],
                            "dist[m]" : compass_observation[0],
                            "sinYaw" : compass_observation[1],
                            "cosYaw" : compass_observation[2]})
                    df_goal.to_csv(os.path.join(sensor_dir,"goalLog.txt"), mode="a", index=False, header=it==0)

                    # reference of the images correespoinding to each control input
                    df_rgb = pd.DataFrame({"timestamp[ns]"  : time_data_buffer[it],
                            "frame" : int(frame_data_buffer[it])})
                    df_rgb.to_csv(os.path.join(sensor_dir,"rgbFrames.txt"), mode="a", index=False, header=it==0)

                    # raw pose data (for debug purposes and (also) to prevent one from having to re-run the data collection in case of a deg2rad issue...)
                    df_pose = pd.DataFrame({"timestamp[ns]"  : time_data_buffer[it],
                            "x[cm]" : state_data_buffer[it][0],
                            "y[cm]" : state_data_buffer[it][1],
                            "z[cm]" : state_data_buffer[it][2],
                            "pitch[rad]" : state_data_buffer[it][3],
                            "yaw[rad]" : state_data_buffer[it][4],
                            "roll[rad]" : state_data_buffer[it][5]})
                    df_pose.to_csv(os.path.join(sensor_dir,"poseData.txt"), mode="a", index=False, header=it==0)

                    # waypoint data (for debug purposes)
                    df_waypoint = pd.DataFrame({"timestamp[ns]"  : time_data_buffer[it],
                            "waypoint_x[cm]" : waypoint_data_buffer[it][0],
                            "waypoint_y[cm]" : waypoint_data_buffer[it][1],
                            "waypoint_z[cm]" : waypoint_data_buffer[it][2]})
                    df_waypoint.to_csv(os.path.join(sensor_dir,"waypointData.txt"), mode="a", index=False, header=it==0)

                if args.create_plot: # if desired, generate a plot of the control performance
                    plot_tracking_performance(state_data_buffer[:executed_iterations][:], waypoint_data_buffer[:executed_iterations][:], sensor_dir)

                if args.create_video: # if desired, generate a video from the collected rgb observations 
                    os.makedirs(video_dir, exist_ok=True)
                    video_name = str(episode) + "_" + scene_id
                    generate_video(config, video_name, image_dir, video_dir, True)

                episode = episode + 1 # update the episode count and move to the next episode 

        # close the current scene
        env.close()

    print("Done.")
