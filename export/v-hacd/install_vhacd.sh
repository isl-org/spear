#!/bin/bash

cd app
cmake CMakeLists.txt
cmake --build .
sudo ln -s "$PWD/build/TestVHACD" /usr/local/bin/TestVHACD
