#!/bin/bash
cd

#Setup esp-idf according to https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/linux-macos-setup.html

#Get packages for Ubuntu
apt-get install git wget flex bison gperf python3 python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0

#This is where you would install python 3. It should already be installed if you have ros2

#Create ~/esp and download esp-idf into ~/esp/esp-idf
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git

#Install tools for esp32s3. Change to match your specific chip
cd ~/esp/esp-idf
./install.sh esp32s3


