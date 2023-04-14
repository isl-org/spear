#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import argparse
import fnmatch
import glob
import os
import posixpath
import spear
import subprocess
import sys


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--unreal_engine_dir", required=True)    
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--skip_create_symlinks", action="store_true")
    parser.add_argument("--perforce_content_dir")
    parser.add_argument("--scene_ids")
    args = parser.parse_args()

    assert os.path.exists(args.unreal_engine_dir)

    unreal_project_dir         = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "cpp", "unreal_projects", "SpearSim"))
    uproject                   = os.path.realpath(os.path.join(unreal_project_dir, "SpearSim.uproject"))
    unreal_project_content_dir = os.path.realpath(os.path.join(unreal_project_dir, "Content"))
    output_dir                 = os.path.realpath(args.output_dir)

    if sys.platform == "win32":
        platform          = "Windows"
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UE4Editor.exe"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Win64", "UnrealPak.exe"))
    elif sys.platform == "darwin":
        platform          = "Mac"
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UE4Editor.app", "Contents", "MacOS", "UE4Editor"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Mac", "UnrealPak"))
    elif sys.platform == "linux":
        platform          = "Linux"
        unreal_editor_bin = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UE4Editor"))
        unreal_pak_bin    = os.path.realpath(os.path.join(args.unreal_engine_dir, "Engine", "Binaries", "Linux", "UnrealPak"))
    else:
        assert False

    assert os.path.exists(unreal_editor_bin)
    assert os.path.exists(unreal_pak_bin)

    # once we know the platform, set our cooked dir
    unreal_project_cooked_dir = os.path.realpath(os.path.join(unreal_project_dir, "Saved", "Cooked", platform + "NoEditor"))

    # We use different strategies for setting scene_ids, depending on if we're creating symlinks or not.
    # If we're not creating symlinks, then the user must specify args.scene_ids. If we are creating
    # symlinks, then we get a list of candidate scene_ids from Perforce and optionally filter.
    if args.skip_create_symlinks:
        assert args.scene_ids is not None
        scene_ids = [args.scene_ids]
    else:
        assert os.path.exists(args.perforce_content_dir)
        perforce_content_scenes_dir = os.path.realpath(os.path.join(args.perforce_content_dir, "Scenes"))
        assert os.path.exists(perforce_content_scenes_dir)

        ignore_names = [".DS_STORE"]
        scene_ids = [ os.path.basename(x) for x in os.listdir(perforce_content_scenes_dir) if x not in ignore_names ]
        assert len(scene_ids) > 0
        if args.scene_ids is not None:
            scene_ids = [ s for s in scene_ids if fnmatch.fnmatch(s, args.scene_ids) ]
        assert len(scene_ids) > 0

    # Create a symlink to the Shared directory
    if not args.skip_create_symlinks:
        perforce_content_shared_dir = os.path.realpath(os.path.join(args.perforce_content_dir, "Shared"))
        assert os.path.exists(perforce_content_shared_dir)

        # We do not want to use os.path.realpath(...) here, because that will resolve to the directory inside the user's Perforce workspace.
        # Instead, we want this path to refer to the symlinked version inside the user's unreal project directory.
        unreal_project_content_shared_dir = os.path.join(unreal_project_content_dir, "Shared")

        if spear.path_exists(unreal_project_content_shared_dir):
            print(f"[SPEAR | build_pak_files.py] File or directory or symlink exists, removing: {unreal_project_content_shared_dir}")
            spear.remove_path(unreal_project_content_shared_dir)

        print(f"[SPEAR | build_pak_files.py] Creating symlink: {unreal_project_content_shared_dir} -> {perforce_content_shared_dir}")
        os.symlink(perforce_content_shared_dir, unreal_project_content_shared_dir)

    # We do not want to use os.path.realpath(...) here, because that will resolve to the directory inside the user's Perforce workspace.
    # Instead, we want this path to refer to the symlinked version inside the user's unreal project directory.
    unreal_project_content_scenes_dir = os.path.join(unreal_project_content_dir, "Scenes")
    assert os.path.exists(unreal_project_content_scenes_dir)

    for scene_id in scene_ids:

        pak_dirs = [
            os.path.realpath(os.path.join(unreal_project_cooked_dir, "Engine", "Content")),
            os.path.realpath(os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "Shared")),
            os.path.realpath(os.path.join(unreal_project_cooked_dir, "SpearSim", "Content", "Scenes", scene_id)),
        ]

        txt_file = os.path.realpath(os.path.join(output_dir, scene_id + "-" + args.version_tag + "-" + platform + ".txt"))
        pak_file = os.path.realpath(os.path.join(output_dir, scene_id + "-" + args.version_tag + "-" + platform + ".pak"))

        # We do not want to use os.path.realpath(...) here, because that will resolve to the directory inside the user's Perforce workspace.
        # Instead, we want this path to refer to the symlinked version inside the user's unreal project directory.
        unreal_project_content_scene_dir = os.path.join(unreal_project_content_dir, "Scenes", scene_id)

        if not args.skip_create_symlinks:
            perforce_content_scene_dir = os.path.realpath(os.path.join(perforce_content_scenes_dir, scene_id))

            if spear.path_exists(unreal_project_content_scene_dir):
                print(f"[SPEAR | build_pak_files.py] File or directory or symlink exists, removing: {unreal_project_content_scene_dir}")
                spear.remove_path(unreal_project_content_scene_dir)

            print(f"[SPEAR | build_pak_files.py] Creating symlink: {unreal_project_content_scene_dir} -> {perforce_content_scene_dir}")
            os.symlink(perforce_content_scene_dir, unreal_project_content_scene_dir)

        # Now that we have created a symlink, our Unreal project should contain exactly two scenes: starter_content_0000 and scene_id
        ignore_names = [".DS_STORE"]
        unreal_project_scenes = { os.path.basename(x) for x in os.listdir(unreal_project_content_scenes_dir) if x not in ignore_names }
        assert unreal_project_scenes == {"starter_content_0000", scene_id}

        # see https://docs.unrealengine.com/4.26/en-US/SharingAndReleasing/Deployment/Cooking for more information on these parameters
        cmd = [
            unreal_editor_bin,
            uproject,
            "-run=Cook",
            "-targetplatform=" + platform + "NoEditor",
            "-fileopenlog",
            "-ddc=InstalledDerivedDataBackendGraph",
            "-unversioned",
            "-stdout",
            "-fullstdoutlogoutput",
            "-crashforuat",
            "-unattended",
            "-nologtimes",
            "-utf8output"
        ]
        print(f"[SPEAR | build_pak_files.py] Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        # create the output_dir
        os.makedirs(output_dir, exist_ok=True)

        for i, pak_dir in enumerate(pak_dirs):
            with open(txt_file, mode="w" if i==0 else "a") as f:
                for content_file in glob.glob(os.path.realpath(os.path.join(pak_dir, "**", "*.*")), recursive=True):
                    assert content_file.startswith(unreal_project_cooked_dir)
                    content_file = content_file.replace('\\', "/")
                    mount_file = posixpath.join("..", "..", ".." + content_file.split(platform + "NoEditor")[1])
                    f.write(f'"{content_file}" "{mount_file}" "" \n')

        # construct command to generate the final pak file
        cmd = [
            unreal_pak_bin,
            pak_file,
            "-create=" + txt_file,
            "-platform=" + platform,
            "-multiprocess",
            "-compress"
        ]
        print(f"[SPEAR | build_pak_files.py] Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        assert os.path.exists(pak_file)
        print(f"[SPEAR | build_pak_files.py] Successfully built {pak_file}")

        if not args.skip_create_symlinks:
            print(f"[SPEAR | build_pak_files.py] Removing symlink: {unreal_project_content_scene_dir}")
            spear.remove_path(unreal_project_content_scene_dir)

    if not args.skip_create_symlinks:
        print(f"[SPEAR | build_pak_files.py] Removing symlink: {unreal_project_content_shared_dir}")
        spear.remove_path(unreal_project_content_shared_dir)

    print("[SPEAR | build_pak_files.py] Done.")
