#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

# Please refer to https://medium.com/@parthnaik92/codesign-and-notarize-unreal-engine-mac-builds-for-distribution-outside-of-the-mac-store-d2f6e444f3a7
# for pre-requisites and setup your system before trying to run this file.

import argparse
import os
import shutil
import spear
import subprocess
import sys
import time


if __name__ == "__main__":

    assert sys.platform == "darwin"

    parser = argparse.ArgumentParser()
    parser.add_argument("--developer_id", required=True)
    parser.add_argument("--apple_username", required=True)
    parser.add_argument("--apple_password", required=True)
    parser.add_argument("--version_tag", required=True)
    parser.add_argument("--input_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "build", "SpearSim-Mac-Shipping-Unsigned")))
    parser.add_argument("--output_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "build", "SpearSim-Mac-Shipping")))
    parser.add_argument("--temp_dir", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "..", "tmp")))
    parser.add_argument("--entitlements_file", default=os.path.realpath(os.path.join(os.path.dirname(__file__), "entitlements.plist")))
    parser.add_argument("--wait_time_seconds", type=float, default=600.0)
    parser.add_argument("--request_uuid")
    args = parser.parse_args()

    # make sure output_dir is empty
    shutil.rmtree(args.output_dir, ignore_errors=True)

    # create the temp directory
    os.makedirs(args.temp_dir, exist_ok=True)

    # create a copy of the executable in output_dir and use it throughout this file
    shutil.copytree(args.input_dir, args.output_dir)

    executable = os.path.realpath(os.path.join(args.output_dir, "Mac", "SpearSim-Mac-Shipping.app"))
    assert os.path.exists(executable)

    if not args.request_uuid:
        spear.log("Changing rpaths...")

        # change current working directory to add relative rpaths
        cwd = os.getcwd()
        new_wd = os.path.realpath(os.path.join(executable, ".."))
        spear.log(f"Changing working directory to {new_wd}")
        os.chdir(new_wd)

        executable_name = os.path.basename(executable)

        add_rpath_dirs = [
            os.path.join(executable_name, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Ogg", "Mac"),
            os.path.join(executable_name, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Vorbis", "Mac"),
            os.path.join(executable_name, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac"),
            os.path.join(executable_name, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac")
        ]

        add_rpath_dylibs = [
            os.path.join(executable_name, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Ogg", "Mac", "libogg.dylib"),
            os.path.join(executable_name, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Vorbis", "Mac", "libvorbis.dylib"),
            os.path.join(executable_name, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac", "libtbbmalloc.dylib"),
            os.path.join(executable_name, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac", "libtbb.dylib")
        ]

        for dir, dylib in zip(add_rpath_dirs, add_rpath_dylibs):
            cmd = ["install_name_tool", "-add_rpath", dir, dylib]
            spear.log(f"Executing: {' '.join(cmd)}")
            subprocess.run(cmd, check=True)

        # revert current working directory
        spear.log(f"Changing working directory to {cwd}")
        os.chdir(cwd)

        # files that need to be code-signed
        sign_files = [
            os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Ogg", "Mac", "libogg.dylib")),
            os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Vorbis", "Mac", "libvorbis.dylib")),
            os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac", "libtbbmalloc.dylib")),
            os.path.realpath(os.path.join(executable, "Contents", "UE", "Engine", "Binaries", "ThirdParty", "Intel", "TBB", "Mac", "libtbb.dylib")),
            os.path.realpath(os.path.join(executable, "Contents", "MacOS", os.path.splitext(executable_name)[0]))
        ]

        assert os.path.exists(args.entitlements_file)

        for file in sign_files:
            cmd = [
                "sudo", "codesign", "-f", "-s", "-v", "--options", "runtime", "--timestamp", "--entitlements", 
                args.entitlements_file, "--sign", f"Developer ID Application: {args.developer_id}", file]
            spear.log(f"Executing: {' '.join(cmd)}")
            subprocess.run(cmd, check=True)

        # create a zip file for notarization
        notarization_zip = os.path.realpath(os.path.join(args.temp_dir, f"{os.path.splitext(executable_name)[0]}.zip"))
        cmd = ["ditto", "-c", "-k", "--rsrc", "--keepParent", executable, notarization_zip]
        spear.log(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd, check=True)

        # send the zip file for notarization
        cmd = [
            "xcrun", "altool", "--notarize-app", "--primary-bundle-id", "org.embodiedaifoundation.spear", 
            "--username", args.apple_username, "--password", args.apple_password, "--file", notarization_zip]
        spear.log(f"Executing: {' '.join(cmd)}")
        ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
        request_uuid = ""
        for line in ps.stdout:
            spear.log(f"{line}")
            if "RequestUUID = " in line:
                request_uuid = line.split("RequestUUID = ")[1].strip()
        ps.wait()
        ps.stdout.close()
        assert request_uuid != ""
        spear.log(f"Zip file sent for notarization. Request UUID: {request_uuid}")
    else:
        request_uuid = args.request_uuid

    # check notarization status
    cmd = ["xcrun", "altool", "--notarization-info", request_uuid, "--username", args.apple_username, "--password", args.apple_password]
    spear.log(f"Executing: {' '.join(cmd)}")
    output = "in progress"
    start_time = time.time()
    elapsed_time = time.time() - start_time
    while output == "in progress" and elapsed_time < args.wait_time_seconds:
        ps = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
        for line in ps.stdout:
            spear.log(f"{line}")
            if "Status:" in line:
                output = line.split("     Status:")[1].strip()
        ps.wait()
        ps.stdout.close()
        elapsed_time = time.time() - start_time
        spear.log(f'Waiting to get more information on the notarization request UUID {request_uuid}, current status is "{output}"...')
    
    if elapsed_time > args.wait_time_seconds:
        spear.log(f"Exceeded maximum wait time ({args.wait_time_seconds}s) for request UUID {request_uuid}. Please complete the rest of the procedure after notarization is complete.")
        assert False
            
    # staple the executable
    cmd = ["xcrun", "stapler", "staple", executable]
    spear.log(f"Executing: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)
    
    spear.log(f"{executable} has been successfully signed.")
    spear.log("Done.")
