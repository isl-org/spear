#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# generate series of actions for fetch agent on different parts such as moving, arm and gripper.

import argparse
import os
import numpy as np
import pandas as pd
import spear

DEFAULT_ACTION = {
    "wheel_joint_l.add_to_angular_velocity_target"      : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "wheel_joint_r.add_to_angular_velocity_target"      : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "arm_joint_0.add_to_angular_orientation_target"     : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "arm_joint_1.add_to_angular_orientation_target"     : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "arm_joint_2.add_to_angular_orientation_target"     : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "arm_joint_3.add_to_angular_orientation_target"     : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "arm_joint_4.add_to_angular_orientation_target"     : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "arm_joint_5.add_to_angular_orientation_target"     : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "arm_joint_6.add_to_angular_orientation_target"     : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "gripper_finger_joint_l.add_force"                  : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "gripper_finger_joint_r.add_force"                  : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "head_pan_joint.add_to_angular_orientation_target"  : np.array([0.0, 0.0, 0.0], dtype=np.float64),
    "head_tilt_joint.add_to_angular_orientation_target" : np.array([0.0, 0.0, 0.0], dtype=np.float64),
}

def get_data_frame(action):
    columns = np.array([ [ name + ".x", name + ".y", name + ".z" ] for name, data in action.items() ]).ravel()  # append each name with .x, .y, .z
    data = [ np.array([ [ data[0], data[1], data[2] ] for name, data in action.items() ]).ravel() ]  # get data as list of unraveled N*3 array
    return pd.DataFrame(columns=columns, data=data)


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--actions_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "actions.csv")))
    args = parser.parse_args()

    df = pd.DataFrame()

    # turn left
    for _ in range(12):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([0.098, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([-0.098,  0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # turn right (negates accumulated "turn left" torque values)
    for _ in range(12):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([-0.098, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([0.098,  0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # lift arm_joint_1
    for _ in range(1):
        action = DEFAULT_ACTION.copy()
        action["arm_joint_1.add_to_angular_orientation_target"] = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # turn arm_joint_6
    for _ in range(52):
        action = DEFAULT_ACTION.copy()
        action["arm_joint_6.add_to_angular_orientation_target"] = np.array([0.0, 0.0, 2.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # move forward keeping the gripper open
    for _ in range(18):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([0.103, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([0.103, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_l.add_force"] = np.array([100.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # move back keeping the gripper open (negates accumulated "move forward" torque values)
    for _ in range(18):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([-0.103, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([-0.103, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_l.add_force"] = np.array([100.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # move forward slowly keeping the gripper open
    for _ in range(10):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([0.02, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([0.02, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_l.add_force"] = np.array([100.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # move back slowly keeping the gripper open (negates accumulated "move forward" torque values)
    for _ in range(10):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([-0.02, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([-0.02, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_l.add_force"] = np.array([100.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # grab object
    for _ in range(10):
        action  = DEFAULT_ACTION.copy()
        action["gripper_finger_joint_l.add_force"] = np.array([-500.0, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_r.add_force"] = np.array([-500.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # move back keeping the gripper closed
    for _ in range(25):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([-0.1, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([-0.1, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_l.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_r.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # move forward keeping the gripper closed (negates accumulated "move back" torque values)
    for _ in range(25):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([0.1, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([0.1, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_l.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_r.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # turn right keeping the gripper closed
    for _ in range(28):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([-0.02, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([0.02,  0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_l.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_r.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # turn left keeping the gripper closed (negates accumulated "turn right" torque values)
    for _ in range(28):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([0.02, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([-0.02,  0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_l.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_r.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # move forward keeping the gripper closed
    for _ in range(30):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([0.06, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([0.06,  0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_l.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_r.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # move back keeping the gripper closed (negates accumulated "move forward" torque values)
    for _ in range(30):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([-0.06, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([-0.06,  0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_l.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_r.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # lower arm_joint_1, arm_joint_3, raise arm_joint_5
    for _ in range(25):
        action = DEFAULT_ACTION.copy()
        action["arm_joint_1.add_to_angular_orientation_target"] = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        action["arm_joint_3.add_to_angular_orientation_target"] = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        action["arm_joint_5.add_to_angular_orientation_target"] = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        action["gripper_finger_joint_l.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        action["gripper_finger_joint_r.add_force"] = np.array([-1000.0, 0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # lower arm_joint_1, raise arm_joint_3 and drop the object
    for _ in range(20):
        action = DEFAULT_ACTION.copy()
        action["arm_joint_1.add_to_angular_orientation_target"] = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        action["arm_joint_3.add_to_angular_orientation_target"] = np.array([0.0, 0.0, -1.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # raise arm_joint_3
    for _ in range(20):
        action = DEFAULT_ACTION.copy()
        action["arm_joint_3.add_to_angular_orientation_target"] = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # turn left
    for _ in range(12):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([0.1, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([-0.1,  0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # turn right (negates accumulated "turn left" torque values)
    for _ in range(12):
        action = DEFAULT_ACTION.copy()
        action["wheel_joint_l.add_to_angular_velocity_target"] = np.array([-0.1, 0.0, 0.0], dtype=np.float64)
        action["wheel_joint_r.add_to_angular_velocity_target"] = np.array([0.1,  0.0, 0.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])
    
    # lower arm_joint_1, arm_joint_3, raise arm_joint_5
    for _ in range(25):
        action = DEFAULT_ACTION.copy()
        action["arm_joint_3.add_to_angular_orientation_target"] = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        action["arm_joint_5.add_to_angular_orientation_target"] = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        df = pd.concat([df, get_data_frame(action)])

    # save to csv
    df.to_csv(args.actions_file, float_format="%.5f", mode="w", index=False)

    spear.log("Done.")
