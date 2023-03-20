#!/bin/bash
cd

# Install ROS2 Humble with commands from https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

#Set locale to UTF-8
apt update && apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

#Ensure universe repository is enabled
apt install software-properties-common -y
add-apt-repository universe -y

#Add ROS 2 GPG key with apt
apt update && apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

#Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

#Update and Upgrade packages
apt update && apt upgrade -y

#Install ros-humble-desktop package and set timezone
DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt install ros-humble-desktop -y

#Install colcon to build ROS2 packages
apt install python3-colcon-common-extensions -y

