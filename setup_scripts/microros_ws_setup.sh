#!/bin/bash
cd

#Create microros workspace with commands from https://micro.ros.org/docs/tutorials/core/first_application_linux/

#Source ros2
source /opt/ros/$ROS_DISTRO/setup.bash

#Make microros_ws and clone microros repository into it
mkdir microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

#Make sure pip and rosdep are installed and initialized
apt-get install python3-pip -y
pip install -U rosdep
rosdep init

#Update dependencies using rosdep
apt update && rosdep ugdate -y
rosdep install --from-paths src --ignore-src -y

#Build and source microros tools
colcon build
source install/local_setup.bash
