# Getting Started

## Working with multiple scenes

If you'd like to work with multiple scenes, you will need to download them separately. See our [latest release notes](https://github.com/isl-org/spear/releases/tag/v0.4.0) for download links. You'll also need to use our command-line tools, which means you'll need to follow the steps below.

### Clone this repository including submodules

The first step is to clone this repository including submodules. We have found that the _recurse submodules_ features in some Git applications don't always download submodules as expected. We therefore recommend using the following commands.

```console
git clone --recurse-submodules https://github.com/isl-org/spear path/to/spear

# checkout the code corresponding to a specific release
cd path/to/spear
git checkout v0.4.0
```

### Install the `spear` Python package

The next step is to install the `spear` Python package as follows.

```console
# create environment
conda create --name spear-env python=3.10 tornado==4.5.3
conda activate spear-env

# install pip
conda install -c anaconda pip

# install msgpack-rpc-python separately from other Python dependencies so
# we can use a specific commit from the msgpack-rpc-python GitHub repository
python -m pip install -e third_party/msgpack-rpc-python

# install a specific version of wheel and setuptools to avoid issue compiling gym
# https://github.com/freqtrade/freqtrade/issues/8376#issuecomment-1519257211
python -m pip install -U wheel==0.38.4 setuptools==65.5.0

# install the spear Python package
python -m pip install -e python
```

### Navigate around a specific scene

At this point, you can use our `run_executable.py` command-line tool to select which scene you want to navigate around. If you wanted to explore our `debug_0000` scene, you would use the following command.

```console
# interactively navigate through a specific scene
python tools/run_executable.py --executable path/to/executable --scene_id debug_0000
```

Depending on your platform, you will need to specify the following path to your `--executable`.

```
Windows: path/to/SpearSim-v0.4.0-Win64-Shipping/SpearSim/Binaries/Win64/SpearSim-Win64-Shipping-Cmd.exe
macOS:   path/to/SpearSim-Mac-Shipping.app
Linux:   path/to/SpearSim-v0.4.0-Linux-Shipping/SpearSim.sh
```

You will also need to specify the following command-line arguments.

  - `--scene_id` is the name of the scene you want to navigate around (e.g., `apartment_0000`, `debug_0000`, `kujiale_0000`, `kujiale_0001`, `...`, `warehouse_0000`). If you specify a `kujiale` or `warehouse` scene, then you also need to specify `--paks_dir` as the directory containing the pak file for that scene. We provide links to pak files in our [release notes](https://github.com/isl-org/spear/releases/tag/v0.4.0).

The following command-line arguments are optional.

  - `--vk_icd_filenames` only has an effect on Linux, and is used to force the Vulkan runtime to load a vendor-specific GPU driver. Our `run_executable.py` script will set the `VK_ICD_FILENAMES` environment variable to whatever is passed into `--vk_icd_filenames`. This argument may or may not be necessary, depending on your specific hardware setup. If you have already set the `VK_ICD_FILENAMES` environment variable before invoking `run_executable.py`, you do not need to specify `--vk_icd_filenames`. If you have an NVIDIA GPU, you probably need to specify `--vk_icd_filenames /usr/share/vulkan/icd.d/nvidia_icd.json`.

## Programmatically interacting with SPEAR via Python

We provide several example applications that demonstrate how to programmatically interact with SPEAR via Python, and highlight what is currently possible with SPEAR. In order to run our example applications, you will need to follow the steps below.

### Configure the behavior of SPEAR

In typical use cases, you will need to configure the behavior of SPEAR before you interact with it. In each of our example applications, we include a configuration file named `user_config.yaml.example` to use as a starting point. To run each example application, you must rename this file to `user_config.yaml` and modify the contents appropriately for your system. In all cases, you will need to set the `SPEAR.STANDALONE_EXECUTABLE` parameter to the location of your `SpearSim` executable (see the note above for which executable to use, depending on your platform). Your `user_config.yaml` file only needs to specify the value of a parameter if it differs from the defaults defined in the `python/config` directory. You can browse this directory for a complete set of all user-configurable parameters.

If you're running on Linux, you may need to set the `SPEAR.VK_ICD_FILENAMES` parameter, which will be used to set the `VK_ICD_FILENAMES` environment variable before launching `SpearSim`. See above for a more detailed discussion.

### Run an example application

You are now ready to run an example application.

```console
python examples/getting_started/run.py
```

We recommend browsing through each of our example applications to get a sense of what is currently possible with SPEAR.
  - [`examples/getting_started`](../examples/getting_started) demonstrates how to control a simple agent and obtain egocentric visual observations.
  - [`examples/generate_image_dataset`](../examples/generate_image_dataset) demonstrates how to generate a dataset of images using our camera agent.
  - [`examples/imitation_learning_openbot`](../examples/imitation_learning_openbot) demonstrates how to collect navigation training data for an OpenBot.
  - [`examples/open_loop_control_fetch`](../examples/open_loop_control_fetch) demonstrates how to control a Fetch robot agent.
