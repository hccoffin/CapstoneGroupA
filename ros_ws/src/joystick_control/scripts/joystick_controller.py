#!/usr/bin/env python3
import time
import sys
import pygame
import rospy

pygame.display.init()
pygame.joystick.init()

print("Found", pygame.joystick.get_count(), "joystick")

_joystick = pygame.joystick.Joystick(0)
_joystick.init()

# Frequency to send control messages at
update_freq = 30 #hz

while(True):
	pygame.event.pump()
	roll = _joystick.get_axis(0)
	pitch = _joystick.get_axis(1)
	yaw = _joystick.get_axis(2)
	throttle = _joystick.get_axis(3)
	print("Roll:",roll)
	print("Pitch:",pitch)
	print("Yaw:",yaw)
	print("Throttle:",throttle)
	time.sleep(1.0/update_freq)
