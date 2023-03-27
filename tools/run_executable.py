#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import os
import spear
import subprocess
import sys


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", required=True)
    parser.add_argument("--temp_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "tmp")))
    parser.add_argument("--data_dir")
    parser.add_argument("--scene_id")
    parser.add_argument("--map_id")
    args = parser.parse_args()
    
    assert os.path.exists(args.executable)

    # determine the internal executable we will actually launch
    executable_name, executable_ext = os.path.splitext(args.executable)

    if sys.platform == "win32":
        assert executable_name[-4:] == "-Cmd"
        assert executable_ext == ".exe"
        executable_internal = os.path.realpath(args.executable)
    elif sys.platform == "darwin":
        assert executable_ext == ".app"
        executable_internal = os.path.realpath(os.path.join(args.executable, "Contents", "MacOS", os.path.basename(executable_name)))
    elif sys.platform == "linux":
        assert executable_ext == ".sh"
        executable_internal = os.path.realpath(args.executable)
    else:
        assert False

    assert os.path.exists(executable_internal)

    # load config
    config = spear.get_config(user_config_files=[])
    config.defrost()
    config.SIMULATION_CONTROLLER.INTERACTION_MODE = "interactive"
    if args.scene_id is not None:
        config.SIMULATION_CONTROLLER.SCENE_ID = args.scene_id
    if args.map_id is not None:
        config.SIMULATION_CONTROLLER.MAP_ID = args.map_id
    config.freeze()

    # write temp file
    temp_dir = os.path.realpath(args.temp_dir)
    temp_config_file = os.path.realpath(os.path.join(temp_dir, "config.yaml"))

    print("[SPEAR | run_executable.py] Writing temp config file: " + temp_config_file)

    if not os.path.exists(temp_dir):
        os.makedirs(temp_dir)
    with open(temp_config_file, "w") as output:
        config.dump(stream=output, default_flow_style=False)

    # create a symlink to data_dir
    if args.data_dir is not None:

        assert os.path.exists(args.data_dir)

        if sys.platform == "win32":
            paks_dir = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(args.executable)), "..", "..", "Content", "Paks"))
        elif sys.platform == "darwin":
            paks_dir = os.path.realpath(os.path.join(args.executable, "Contents", "UE4", "SpearSim", "Content", "Paks"))
        elif sys.platform == "linux":
            paks_dir = os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(args.executable)), "SpearSim", "Content", "Paks"))
        else:
            assert False

        assert os.path.exists(paks_dir)

        # we don't use os.path.realpath here because we don't want to resolve the symlink
        spear_data_dir = os.path.join(paks_dir, "SpearData")

        if spear.path_exists(spear_data_dir):
            print(f"[SPEAR | run_executable.py] File or directory or symlink exists, removing: {spear_data_dir}")
            spear.remove_path(spear_data_dir)

        print(f"[SPEAR | run_executable.py] Creating symlink: {spear_data_dir} -> {args.data_dir}")
        os.symlink(args.data_dir, spear_data_dir)

    # provide additional control over which Vulkan devices are recognized by Unreal
    if len(config.SPEAR.VULKAN_DEVICE_FILES) > 0:
        print("[SPEAR | run_executable.py] Setting VK_ICD_FILENAMES environment variable: " + config.SPEAR.VULKAN_DEVICE_FILES)
        os.environ["VK_ICD_FILENAMES"] = config.SPEAR.VULKAN_DEVICE_FILES

    # set up launch executable and command-line arguments
    launch_args = []

    launch_args.append("-game")
    launch_args.append("-windowed")
    launch_args.append("-novsync")
    launch_args.append("-nosound")
    launch_args.append("-notexturestreaming")
    launch_args.append("-resx={}".format(config.SPEAR.WINDOW_RESOLUTION_X))
    launch_args.append("-resy={}".format(config.SPEAR.WINDOW_RESOLUTION_Y))
    launch_args.append("-graphicsadapter={}".format(config.SPEAR.GPU_ID))

    if len(config.SPEAR.UNREAL_INTERNAL_LOG_FILE) > 0:
        launch_args.append("-log={}".format(config.SPEAR.UNREAL_INTERNAL_LOG_FILE))
   
    # on Windows, we need to pass in extra command-line parameters to enable DirectX 12
    # and so that calls to UE_Log and writes to std::cout are visible on the command-line
    if sys.platform == "win32":
        launch_args.append("-dx12")            
        launch_args.append("-stdout")
        launch_args.append("-fullstdoutlogoutput")

    launch_args.append("-config_file={}".format(temp_config_file))

    for a in config.SPEAR.CUSTOM_COMMAND_LINE_ARGUMENTS:
        launch_args.append("{}".format(a))

    # launch executable
    cmd = [executable_internal] + launch_args

    print("[SPEAR | run_executable.py] Launching executable with the following command-line arguments:")
    print(" ".join(cmd))

    print("[SPEAR | run_executable.py] Launching executable with the following config values:")
    print(config)

    subprocess.run(cmd, check=True)
    
    print("[SPEAR | run_executable.py] Done.")
