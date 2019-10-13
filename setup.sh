#!/bin/bash

chmod +x ros_ws/src/joystick_control/scripts/joystick_controller.py

sudo apt update

sudo apt install python3-pip
pip3 install evdev
pip3 install pygame
pip3 install rospkg
