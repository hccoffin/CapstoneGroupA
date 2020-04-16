#!/bin/bash

chmod +x ros_ws/src/UI/joystick_command/scripts/joystick_controller.py

sudo apt update

# Assuming in Conda3 Environment
pip install rospkg
pip install bios
pip install odrive
pip install casadi

pip uninstall em
pip install empy

# ROS libraries
sudo apt install ros-melodic-effort-controllers
sudo apt install ros-melodic-joint-state-controllers
sudo apt install ros-melodic-joint-state-publishers
sudo apt install ros-melodic-tf2
sudo apt install ros-melodic-tf2-geometry-msgs

cd ros_ws
catkin_make
source devel/setup.bash
