#!/bin/bash

chmod +x ros_ws/src/UI/joystick_command/scripts/joystick_controller.py

sudo apt update

# Assuming in Conda3 Environment
pip install pygame
pip install rospkg
pip install bios
pip install odrive

# ROS libraries
sudo apt install ros-melodic-effort-controllers
sudo apt install ros-melodic-aruco-detect

cd ros_ws
catkin_make
source devel/setup.bash
