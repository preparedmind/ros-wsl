#!/bin/bash
#Source ros2 humble
cd
source /opt/ros/humble/setup.bash
#Create ros2_ws
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
#Create the package
ros2 pkg create --build-type ament_python key_pub
cp ~/ros-wsl/key_pub/key_pub.py ~/ros2_ws/src/key_pub/key_pub
# Build and source the workspace
cd ~/ros2_ws