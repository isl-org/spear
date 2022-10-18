import argparse
import os 
import shutil
import subprocess
import sys

SCRIPT_DIR_PATH = os.path.dirname(os.path.abspath(__file__))
MIN_CMAKE_VERSION = "3.5.1"

def check_cmake_version():
    min_cmake_version = MIN_CMAKE_VERSION.split(".")
    print(f"cmake min version required is {MIN_CMAKE_VERSION}, checking if this requirement is met...")
    cmake_version = (subprocess.run(["cmake", "--version"], stdout=subprocess.PIPE).stdout).decode("utf-8").split('\n')[0].split(' ')[-1]
    print(f"cmake version on this sytem is {cmake_version}")
    cmake_version = cmake_version.split(".")
    assert len(min_cmake_version) == len(cmake_version)
    for i in range(len(min_cmake_version)):
        if int(cmake_version[i]) < int(min_cmake_version[i]):
            assert False
        elif int(cmake_version[i]) > int(min_cmake_version[i]):
            break
    print("cmake version looks good...")

def build_libs(args):

    if sys.platform == "linux":
        os.environ["CC"] = args.clang_cc_bin
        os.environ["CXX"] = args.clang_cxx_bin

    #
    # rbdl
    #

    print("building rbdl...")
    build_dir = os.path.join(SCRIPT_DIR_PATH, "..", "third_party", "rbdl", "build")
    if os.path.isdir(build_dir):
        shutil.rmtree(build_dir, ignore_errors=True)

    os.makedirs(build_dir)
    os.chdir(build_dir)

    if sys.platform == "linux":
        cmake_args = ["cmake", "-DCMAKE_BUILD_TYPE=Release" , "-DRBDL_BUILD_STATIC=ON", "-DRBDL_BUILD_ADDON_URDFREADER=ON", "-DCMAKE_CXX_FLAGS='-stdlib=libc++'", "-DCMAKE_POSITION_INDEPENDENT_CODE=ON", ".."]
        print(f"Executing cmd: {' '.join(cmake_args)}")
        cmake_cmd = subprocess.run(cmake_args)
        assert cmake_cmd.returncode == 0
        cmake_args = ["cmake",  "--build", ".", "-j", "{0}".format(args.num_parallel_jobs)]
    elif sys.platform == "darwin":
        cmake_args = ["cmake", "-DCMAKE_BUILD_TYPE=Release" , "-DRBDL_BUILD_STATIC=ON", "-DRBDL_BUILD_ADDON_URDFREADER=ON", ".."]
        print(f"Executing cmd: {' '.join(cmake_args)}")
        cmake_cmd = subprocess.run(cmake_args)
        assert cmake_cmd.returncode == 0
        cmake_args = ["cmake",  "--build", ".", "-j", "{0}".format(args.num_parallel_jobs)]
    elif sys.platform == "win32":
        cmake_args = ["cmake", "-DCMAKE_BUILD_TYPE=Release" , "-DRBDL_BUILD_STATIC=ON", "-DRBDL_BUILD_ADDON_URDFREADER=ON",  "-DCMAKE_CXX_FLAGS='/bigobj'", ".."]
        print(f"Executing cmd: {' '.join(cmake_args)}")
        cmake_cmd = subprocess.run(cmake_args)
        assert cmake_cmd.returncode == 0
        cmake_args = ["cmake",  "--build", ".", "--config", "Release", "-j", "{0}".format(args.num_parallel_jobs)]
    else:
        assert False, "This OS is not supported."
    
    print(f"Executing cmd: {' '.join(cmake_args)}")
    cmake_cmd = subprocess.run(cmake_args)
    assert cmake_cmd.returncode == 0
    print("rbdl built successfully...")

    #
    # rpclib
    #

    print("building rpclib...")
    build_dir = os.path.join(SCRIPT_DIR_PATH, "..", "third_party", "rpclib", "build")
    if os.path.isdir(build_dir):
        shutil.rmtree(build_dir, ignore_errors=True)

    os.makedirs(build_dir)
    os.chdir(build_dir)

    if sys.platform == "linux":
        cmake_args = ["cmake", "-DCMAKE_BUILD_TYPE=Release" , "-DCMAKE_CXX_FLAGS='-stdlib=libc++'", "-DCMAKE_POSITION_INDEPENDENT_CODE=ON", ".."]
        print(f"Executing cmd: {' '.join(cmake_args)}")
        cmake_cmd = subprocess.run(cmake_args)
        assert cmake_cmd.returncode == 0
        cmake_args = ["cmake",  "--build", ".", "-j", "{0}".format(args.num_parallel_jobs)]
    elif sys.platform == "darwin":
        cmake_args = ["cmake", "-DCMAKE_BUILD_TYPE=Release", ".."]
        print(f"Executing cmd: {' '.join(cmake_args)}")
        cmake_cmd = subprocess.run(cmake_args)
        assert cmake_cmd.returncode == 0
        cmake_args = ["cmake",  "--build", ".", "-j", "{0}".format(args.num_parallel_jobs)]
    elif sys.platform == "win32":
        cmake_args = ["cmake", "-DCMAKE_BUILD_TYPE=Release", ".."]
        print(f"Executing cmd: {' '.join(cmake_args)}")
        cmake_cmd = subprocess.run(cmake_args)
        assert cmake_cmd.returncode == 0
        cmake_args = ["cmake",  "--build", ".", "--config", "Release", "-j", "{0}".format(args.num_parallel_jobs)]
    else:
        assert False, "This OS is not supported."

    print(f"Executing cmd: {' '.join(cmake_args)}")
    cmake_cmd = subprocess.run(cmake_args)
    assert cmake_cmd.returncode == 0
    print("rpclib built successfully...")

    #
    # yamp-cpp
    #

    print("building yaml-cpp...")
    build_dir = os.path.join(SCRIPT_DIR_PATH, "..", "third_party", "yaml-cpp", "build")
    if os.path.isdir(build_dir):
        shutil.rmtree(build_dir, ignore_errors=True)

    os.makedirs(build_dir)
    os.chdir(build_dir)

    if sys.platform == "linux":
        cmake_args = ["cmake", "-DCMAKE_BUILD_TYPE=Release" , "-DCMAKE_CXX_FLAGS='-stdlib=libc++'", "-DCMAKE_POSITION_INDEPENDENT_CODE=ON", ".."]
        print(f"Executing cmd: {' '.join(cmake_args)}")
        cmake_cmd = subprocess.run(cmake_args)
        assert cmake_cmd.returncode == 0
        cmake_args = ["cmake",  "--build", ".", "-j", "{0}".format(args.num_parallel_jobs)]
    elif sys.platform == "darwin":
        cmake_args = ["cmake", "-DCMAKE_BUILD_TYPE=Release", ".."]
        print(f"Executing cmd: {' '.join(cmake_args)}")
        cmake_cmd = subprocess.run(cmake_args)
        assert cmake_cmd.returncode == 0
        cmake_args = ["cmake",  "--build", ".", "-j", "{0}".format(args.num_parallel_jobs)]
    elif sys.platform == "win32":
        cmake_args = ["cmake", "-DCMAKE_BUILD_TYPE=Release", ".."]
        print(f"Executing cmd: {' '.join(cmake_args)}")
        cmake_cmd = subprocess.run(cmake_args)
        assert cmake_cmd.returncode == 0
        cmake_args = ["cmake",  "--build", ".", "--config", "Release", "-j", "{0}".format(args.num_parallel_jobs)]
    else:
        assert False, "This OS is not supported."
    
    print(f"Executing cmd: {' '.join(cmake_args)}")
    cmake_cmd = subprocess.run(cmake_args)
    assert cmake_cmd.returncode == 0
    print("yaml-cpp built successfully...")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_parallel_jobs", "-n", type=int, default=1, required=False)
    parser.add_argument("--clang_cc_bin", "-ccb", default="clang", required=False)
    parser.add_argument("--clang_cxx_bin", "-cxxb", default="clang++", required=False)
    args = parser.parse_args()

    # check cmake version requirement
    check_cmake_version()

    # build third party libs
    build_libs(args)
