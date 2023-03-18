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
