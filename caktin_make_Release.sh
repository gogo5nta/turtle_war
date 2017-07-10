#!/bin/bash
echo "catkin_make Release"
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
