# ros-wsl

1. Setup clean Ubuntu distro (and way to do that whenever needed):
    - run a docker container so that you have something to export:
    ```
    docker run -t ubuntu:22.04 ls /
    ```
    - find you docker container using `docker ps -a`
    - export your docker container using name or container id
    ```
    docker export ab39032169ee --output C:\Vadim\Projects\11-Robotics\ubuntu-22.02-vp.tar
    ```
    - Create new distribution using wsl
    ```
    cd C:\Vadim\projects\11-Robotics\
    wsl --import ros-humble-ubuntu-22.04 C:\Vadim\projects\11-Robotics\ .\ubuntu-22.02-vp.tar 
    ```
    - check that you now have a new distribution using `wsl --list`.  

    - set your new distribution as default for now
    using `wsl --set-default ros-humble-ubuntu-22.04`

2. Install git and setup ros2 humble and micro ros on the new wsl distrobution:
    - Start the new wsl distrobution using `wsl -d <distro name>`
    - Install git and clone this repository
    - Run the bash script to install ros2 humble and microros and setup key_pub ros2 node and motor_controller micro ros program `bash ~/ros-wsl/setupscripts/run_all.sh`
    - Add the following lines to package.xml
    ```
    <exec_depend>rclpy</exec_depend>
    <exec_depend>std_msgs</exec_depend>
    ```
    - Add the following line to setup.py
    ```
    "keynode = key_pub.key_pub:main",
    ```
    - Build the package in `~/ros2_ws` with `colcon build --packages-select key_pub`
    - Source the package with `source install/local_setup.bash`
    - You can now run the ros2 package with `ros2 run key_pub keynode`
    