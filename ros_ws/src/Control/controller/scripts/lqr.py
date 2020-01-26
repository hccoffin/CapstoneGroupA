#!/usr/bin/env python3

# General Imports
import time
import datetime
import os

# ROS Imports
import rospy
from planner.msg import PlanWithVel
from controller.msg import WheelEst
from sensor_msgs.msg import Imu

# Linear Algebra imports
import numpy as np

# IO imports
import odrive

class Controller(object):
	def __init__(self, _K, _dt, _sim = True):

		# Subscribers
		rospy.Subscriber("/roboassistant/nav_plan", PlanWithVel,self.planCallback)
		rospy.Subscriber("/roboassistant/orientation", Imu, self.imuCallback)
		
		# Publishers
		self.wheel_pub = rospy.Publisher("/roboassistant/wheel_estimate", WheelEst, queue_size = 1)
		
		self.plan = None
		self.frequency = 1.0/_dt
		self.K = _K
		self.sim = _sim

	def imuCallback(self, newImuMsg):
		self.imu_msg = newImuMsg

	def planCallback(self, newPlanMsg):
		self.plan = newPlanMsg

	def getMotorCurrent(self, tau_des):
		return np.array([0, 0]).T

	def loop(self):
		rate = rospy.Rate(self.frequency) # ROS sleep Rate
		while not rospy.is_shutdown():
			# Get encoder feedback from ODrive

			# Assemble state estimate and state error
			state_est = np.zeros([6,1])
			state_des = np.zeros([6,1])
			if (self.plan != None):
				state_des = np.zeros([6, 1])

			# Convert state error to torques
			state_error = state_des - state_est
			tau_des = -np.dot(self.K,state_error)

			if (self.sim):
				# Send torques to gazebo
				pass
			else:
				# Apply torques to ODrive using motor model
				current_des = self.getMotorCurrent(tau_des)

			# Send out new encoder data
			wheel_msg = WheelEst()
			self.wheel_pub.publish(wheel_msg)

			# Enforce constant rate
			rate.sleep()

if __name__ == '__main__':
  # Logging level DEBUG, INFO, WARN, ERROR, FATAL
  rospy.init_node('controller',log_level=rospy.DEBUG)

	# Load K, dt from ROS parameter server
  K = np.array(rospy.get_param('controller/lqr_gains')).reshape((2,6))
  rospy.loginfo("Loaded Gains: ")
  rospy.loginfo(K)

  dt = rospy.get_param('dt')
  rospy.loginfo("Loaded dt: " + str(dt))

  c = Controller(K, dt)
  c.loop()