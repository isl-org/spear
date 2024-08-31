#
# Copyright(c) 2022 Intel. Licensed under the MIT License <http://opensource.org/licenses/MIT>.
#

import setuptools

setuptools.setup(
    name="spear-sim",
    version="0.5.0",
    author="",
    author_email="",
    description="",
    packages=["spear"],
    install_requires=[
        "coacd==1.0.0",
        "ffmpeg-python==0.2.0",
        "gym==0.23.0",
        "matplotlib==3.6.2",
        "mayavi==4.8.2",
        "mujoco==3.1.3",
        "numpy==1.22.3",
        "opencv-python==4.8.1.78",
        "pandas==1.5.1",
        "psutil==5.9.0",
        "scipy==1.10.1",
        "tensorflow==2.13.0",
        "trimesh==4.4.0",
        "vtk==9.2.6",
        "wget==3.2",
        "yacs==0.1.8"])
