#!/bin/bash
#move to microros workspace and source microros and ros2
cd ~/microros_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
#create freertos microros firmware for esp32
ros2 run micro_ros_setup create_firmware.sh freertos esp32
#Cppy motor controller code from local repository to the microros workspace
cp -r ~/ros-wsl/motor_controller ~/microros_ws/firmware/freertos_apps/apps
#configure and build motor controller firmware
ros2 run micro_ros_setup configure_firmware.sh motor_controller --transport serial
ros2 run micro_ros_setup build_firmware.sh

