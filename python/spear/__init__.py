#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

__version__ = "0.5.0"

import os
import sys
from yacs.config import CfgNode

from spear.log import log, log_current_function, log_no_prefix, log_get_prefix
from spear.engine_service import EngineService
from spear.env import Env
from spear.instance import Instance
from spear.legacy_service import LegacyService
from spear.game_world_service import GameWorldService
from spear.path import path_exists, remove_path


spear_root_dir = os.path.dirname(os.path.realpath(__file__))
# ordered from low-level to high-level
default_config_files = [
    os.path.realpath(os.path.join(spear_root_dir, "config", "default_config.sp_core.yaml")),
    os.path.realpath(os.path.join(spear_root_dir, "config", "default_config.vehicle.yaml")),
    os.path.realpath(os.path.join(spear_root_dir, "config", "default_config.urdf_robot.yaml")),
    os.path.realpath(os.path.join(spear_root_dir, "config", "default_config.sp_engine.yaml")),
    os.path.realpath(os.path.join(spear_root_dir, "config", "default_config.spear.yaml")) ]

# This function returns a config object, obtained by loading and merging a list of config
# files in the order they appear in the user_config_files input argument. This function is
# useful for loading default values from multiple different components and systems, and then
# overriding some of the default values with experiment-specific or user-specific overrides.
# Before loading any of the config files specified in user_config_files, all the default
# values required by the spear Python package and C++ plugins are loaded, and can be
# overridden by any of the files appearing in user_config_files.
def get_config(user_config_files):

    # create a single CfgNode that will eventually contain data from all config files
    config = CfgNode(new_allowed=True)

    for c in default_config_files:
        config.merge_from_file(c)

    # In some cases, we need to update specific config values with information that is available
    # at runtime, but isn't available when we are authoring our default_config.*.yaml files.
    config.URDF_ROBOT.URDF_ROBOT_PAWN.URDF_DIR = os.path.realpath(os.path.join(spear_root_dir, "urdf"))

    for c in user_config_files:
        config.set_new_allowed(True)  # required to override an empty dict with a non-empty dict
        config.merge_from_file(c)

    config.freeze()

    return config

def configure_system(config):

    # create a symlink to SPEAR.PAKS_DIR
    if config.SPEAR.LAUNCH_MODE == "standalone" and config.SPEAR.PAKS_DIR != "":

        assert os.path.exists(config.SPEAR.STANDALONE_EXECUTABLE)
        assert os.path.exists(config.SPEAR.PAKS_DIR)

        if sys.platform == "win32":
            paks_dir = \
                os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(config.SPEAR.STANDALONE_EXECUTABLE)), "..", "..", "Content", "Paks"))
        elif sys.platform == "darwin":
            paks_dir = \
                os.path.realpath(os.path.join(config.SPEAR.STANDALONE_EXECUTABLE, "Contents", "UE", "SpearSim", "Content", "Paks"))
        elif sys.platform == "linux":
            paks_dir = \
                os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(config.SPEAR.STANDALONE_EXECUTABLE)), "SpearSim", "Content", "Paks"))
        else:
            assert False

        assert os.path.exists(paks_dir)

        # we don't use os.path.realpath here because we don't want to resolve the symlink
        spear_paks_dir = os.path.join(paks_dir, "SpearPaks")

        if path_exists(spear_paks_dir):
            log(f"File or directory or symlink exists, removing: {spear_paks_dir}")
            remove_path(spear_paks_dir)

        log(f"Creating symlink: {spear_paks_dir} -> {config.SPEAR.PAKS_DIR}")
        os.symlink(config.SPEAR.PAKS_DIR, spear_paks_dir)

    # provide additional control over which Vulkan devices are recognized by Unreal
    if config.SPEAR.ENVIRONMENT_VARS.VK_ICD_FILENAMES != "":
        log("Setting VK_ICD_FILENAMES environment variable: " + config.SPEAR.ENVIRONMENT_VARS.VK_ICD_FILENAMES)
        os.environ["VK_ICD_FILENAMES"] = config.SPEAR.ENVIRONMENT_VARS.VK_ICD_FILENAMES
