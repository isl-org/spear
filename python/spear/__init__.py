#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

__version__ = "0.5.0"

import os
import sys
from yacs.config import CfgNode

from spear.log import log, log_current_function, log_no_prefix, log_get_prefix
from spear.path import path_exists, remove_path
from spear.engine_service import EngineService
from spear.env import Env
from spear.instance import Instance
from spear.legacy_service import LegacyService
from spear.game_world_service import GameWorldService

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

    # create a symlink to SPEAR.INSTANCE.PAKS_DIR
    if config.SPEAR.INSTANCE.LAUNCH_MODE == "standalone" and config.SPEAR.INSTANCE.PAKS_DIR != "":

        assert os.path.exists(config.SPEAR.INSTANCE.STANDALONE)
        assert os.path.exists(config.SPEAR.INSTANCE.PAKS_DIR)

        if sys.platform == "win32":
            paks_dir = \
                os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(config.SPEAR.INSTANCE.STANDALONE)), "..", "..", "Content", "Paks"))
        elif sys.platform == "darwin":
            paks_dir = \
                os.path.realpath(os.path.join(config.SPEAR.INSTANCE.STANDALONE, "Contents", "UE", "SpearSim", "Content", "Paks"))
        elif sys.platform == "linux":
            paks_dir = \
                os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(config.SPEAR.INSTANCE.STANDALONE)), "SpearSim", "Content", "Paks"))
        else:
            assert False

        assert os.path.exists(paks_dir)

        # we don't use os.path.realpath here because we don't want to resolve the symlink
        spear_paks_dir = os.path.join(paks_dir, "SpearPaks")

        if path_exists(spear_paks_dir):
            log(f"File or directory or symlink exists, removing: {spear_paks_dir}")
            remove_path(spear_paks_dir)

        log(f"Creating symlink: {spear_paks_dir} -> {config.SPEAR.INSTANCE.PAKS_DIR}")
        os.symlink(config.SPEAR.INSTANCE.PAKS_DIR, spear_paks_dir)

    # provide additional control over which Vulkan devices are recognized by Unreal
    if config.SPEAR.INSTANCE.VK_ICD_FILENAMES != "":
        log("Setting VK_ICD_FILENAMES environment variable: " + config.SPEAR.INSTANCE.VK_ICD_FILENAMES)
        os.environ["VK_ICD_FILENAMES"] = config.SPEAR.INSTANCE.VK_ICD_FILENAMES

# high-level functions
def begin_tick(instance):
    instance.engine_service.begin_tick()
    instance.game_world_service.unpause_game()

def tick(instance):
    instance.engine_service.tick()

def end_tick(instance):
    instance.game_world_service.pause_game()
    instance.engine_service.end_tick()

def open_level(instance, scene_id, map_id=""):
    desired_level_name = ""
    if scene_id != "":
        if map_id == "":
            map_id = scene_id
        else:
            map_id = map_id
        desired_level_name = "/Game/Scenes/" + scene_id + "/Maps/" + map_id

    log("scene_id:           ", scene_id)
    log("map_id:             ", map_id)
    log("desired_level_name: ", desired_level_name)

    begin_tick(instance)
    current_scene_id = instance.game_world_service.get_current_level()
    instance.game_world_service.open_level(desired_level_name)
    tick(instance)
    end_tick(instance)

    while current_scene_id != scene_id:
        begin_tick(instance)
        current_scene_id = instance.game_world_service.get_current_level()
        tick(instance)
        end_tick(instance)
