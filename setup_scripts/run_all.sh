#!/bin/bash

#Install ros2 humble
bash humble_setup.sh
#Install esp-idf and microros esp-idf components
#esp-idf is included in microros and not necessary to set up any more.
#bash esp-idf_setup.sh
#Install microros and create a workspace
bash microros_ws_setup.sh
#Setup key_pub ros2 package
bash key_pub_setup.sh
#Setup motor_controller microros folder
bash motor_controller_setup.sh