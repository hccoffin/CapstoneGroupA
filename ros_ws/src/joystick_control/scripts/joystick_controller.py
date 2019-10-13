#!/usr/bin/env python3
import time
import sys
import pygame
import rospy
import std_msgs

# Initialize pygame (handles joystick input)
pygame.display.init()
pygame.joystick.init()
print("Found", pygame.joystick.get_count(), "joystick")

# Initialize joystick
_joystick = pygame.joystick.Joystick(0)
_joystick.init()

# Initialize ROS node
rospy.init_node("joystick_controller")

# Frequency to send control messages at
update_freq = rospy.Rate(30) #hz

# Setup ROS publishers
roll_pub = rospy.Publisher('/robot/inputs/roll', std_msgs.msg.Float64, queue_size=1)
pitch_pub = rospy.Publisher('/robot/inputs/pitch', std_msgs.msg.Float64, queue_size=1)
yaw_pub = rospy.Publisher('/robot/inputs/yaw', std_msgs.msg.Float64, queue_size=1)
throttle_pub = rospy.Publisher('/robot/inputs/throttle', std_msgs.msg.Float64, queue_size=1)

# Publish joystick messages to specified ROS topics
print("Sending out joystick messages...")
while not rospy.is_shutdown():
	pygame.event.pump()
	roll = _joystick.get_axis(0)
	pitch = _joystick.get_axis(1)
	yaw = _joystick.get_axis(2)
	throttle = _joystick.get_axis(3)
	roll_pub.publish(roll)
	pitch_pub.publish(pitch)
	yaw_pub.publish(yaw)
	throttle_pub.publish(throttle)
	
	update_freq.sleep()
