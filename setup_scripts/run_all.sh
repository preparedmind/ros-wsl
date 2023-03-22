#!/bin/bash

#Install ros2 humble
bash humble_setup.sh
#Install microros and create a workspace
bash microros_ws_setup.sh
#Install esp-idf and microros esp-idf components
bash esp-idf_setup.sh