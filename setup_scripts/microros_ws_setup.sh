#!/bin/bash
cd

#Create microros workspace with commands from https://micro.ros.org/docs/tutorials/core/first_application_linux/

#Source ros2 humble
source /opt/ros/humble/setup.bash

#Make microros_ws and clone microros repository into it
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

#Make sure pip and rosdep are installed and initialized
apt-get install python3-pip -y
pip install setuptools==58.2.0
apt-get install python3-rosdep -y
rosdep init

#Update dependencies using rosdep
apt update && rosdep update -y
rosdep install --from-paths src --ignore-src -y

#Build and source microros tools
colcon build
source install/local_setup.bash
