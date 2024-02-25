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
        "ffmpeg-python==0.2.0",
        "gym==0.21.0",
        "matplotlib==3.6.2",
        "mayavi==4.8.1",
        "numpy==1.22.3",
        "opencv-python==4.5.5.64",
        "pandas==1.5.1",
        "psutil==5.9.0",
        "pyglet==1.5.0",
        "tensorflow==2.13.0",
        "trimesh[easy]==4.1.4",
        "yacs==0.1.8",
        "wxPython==4.2.1"
    ]
)
