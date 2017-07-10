#!/bin/bash
echo "catkin_make Debug"
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Debug
