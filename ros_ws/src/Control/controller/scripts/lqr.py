#!/usr/bin/env python3

# General Imports
import time
import datetime
import os

# ROS Imports
import rospy
from planner_msgs.msg import PlanWithVel
from controller.msg import WheelEst
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

# Linear Algebra imports
import numpy as np

# IO imports
import odrive

class Controller(object):
	def __init__(self, _K, _dt, _sim = True):

		# Subscribers
		rospy.Subscriber("/planner/current_plan", PlanWithVel,self.planCallback)
<<<<<<< HEAD
		
		# Publishers
		self.wheel_pub = rospy.Publisher("/controller/wheel_estimate", WheelEst, queue_size = 1)
		self.imu_pub = rospy.Publisher("/controller/imu", Imu, queue_size = 1)
		
=======
	
		# Publishers
		if (_sim):
			self.left_torque_pub = rospy.Publisher("/controller/left_torque_controller/command", Float64, queue_size=1)
			self.right_torque_pub = rospy.Publisher("/controller/right_torque_controller/command", Float64, queue_size=1)

		self.wheel_pub = rospy.Publisher("/controller/wheel_estimate", WheelEst, queue_size = 1)
		self.imu_pub = rospy.Publisher("/controller/imu", Imu, queue_size = 1)

>>>>>>> 53febaa361dcc91395d9c77874a7c14d4fbb4077
		self.plan = None
		self.frequency = 1.0/_dt
		self.K = _K
		self.sim = _sim

<<<<<<< HEAD
=======

>>>>>>> 53febaa361dcc91395d9c77874a7c14d4fbb4077
	def planCallback(self, newPlanMsg):
		self.plan = newPlanMsg

	def getMotorCurrent(self, tau_des):
		return np.array([0, 0]).T

	def loop(self):
		rate = rospy.Rate(self.frequency) # ROS sleep Rate
		while not rospy.is_shutdown():
			# Get encoder feedback from ODrive

			# Assemble state estimate and state error
			state_est = np.zeros([4,1])
			state_des = np.zeros([4,1])
			if (self.plan != None):
				state_des = np.zeros([4, 1])

			# Convert state error to torques
			state_error = state_des - state_est
			F_des = -np.matmul(self.K,state_error)

<<<<<<< HEAD
      # Convert F_des to torques for each wheel, add/subtract to turn
			#print(F_des)
=======
			# Convert F_des to tau1 and tau2
			left_torque = 0.0
			right_torque = 0.0
>>>>>>> 53febaa361dcc91395d9c77874a7c14d4fbb4077

			if (self.sim):
				# Send torques to gazebo
				self.left_torque_pub.publish(left_torque)
				self.right_torque_pub.publish(right_torque)
				
			else:
				# Convert desired torques to motors
				current_des = self.getMotorCurrent(tau_des)
        # Apply torque to odrive here

			# Send out new encoder data
			wheel_msg = WheelEst()
			self.wheel_pub.publish(wheel_msg)

<<<<<<< HEAD
      # Send out imu data
=======
			# Send out imu data
>>>>>>> 53febaa361dcc91395d9c77874a7c14d4fbb4077
			imu_msg = Imu()
			self.imu_pub.publish(imu_msg)

			# Enforce constant rate
			rate.sleep()

if __name__ == '__main__':
  # Logging level DEBUG, INFO, WARN, ERROR, FATAL
  rospy.init_node('controller',log_level=rospy.DEBUG)

	# Load K, dt from ROS parameter server
  K = np.array(rospy.get_param('controller/lqr_gains')).reshape((1,4))
  rospy.loginfo("Loaded Gains: ")
  rospy.loginfo(K)

  dt = rospy.get_param('dt')
  rospy.loginfo("Loaded dt: " + str(dt))

  c = Controller(K, dt)
  c.loop()
