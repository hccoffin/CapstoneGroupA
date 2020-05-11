#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Twist
from sensor_msgs.msg import JointState
import angles
import tf
import numpy as np
from traj_opt_casadi import solve_traj_opt

import odrive
from odrive.enums import *
from odrive.utils import *

wheelradius = 0.095
v_max = 0.5
w_max = v_max / wheelradius
wheelbase = 0.2

def location_and_vel_from_odometry(msg):
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	quaternion = msg.pose.pose.orientation
	euler_angles = tf.transformations.euler_from_quaternion(
		[quaternion.x, quaternion.y, quaternion.z, quaternion.w]
	)
	th = euler_angles[2]
	if np.abs(np.cos(th)) < 1e-5:
		v = msg.twist.twist.linear.y / np.sin(th)
	else:
		v = msg.twist.twist.linear.x / np.cos(th)
	omega = msg.twist.twist.angular.z
	return [x, y, th, v, omega]

def location_from_pose(pose):
	x = pose.position.x
	y = pose.position.y
	quaternion = pose.orientation
	euler_angles = tf.transformations.euler_from_quaternion(
		[quaternion.x, quaternion.y, quaternion.z, quaternion.w]
	)
	th = euler_angles[2]
	return [x, y, th]

class ControllerNode(object):

	def __init__(self, odrv0):
		self.path_publisher = rospy.Publisher('/controller/desired_path', Path, queue_size=1)
	
		# TODO: don't set these variables (wait for subscriber call)
		self.robot_loc = [0, 0, 0]
		self.v = 0
		self.omega = 0

		self.odrv0 = odrv0

		self.t = np.array([])
		self.x = np.array([])
		self.y = np.array([])
		self.th = np.array([])
		self.w_left = np.array([])
		self.w_right = np.array([])

		self.start_time = rospy.get_time()

	def goal_pose_callback(self, msg):
		self.odrv0.axis1.controller.vel_setpoint = 0
		self.odrv0.axis0.controller.vel_setpoint = 0
		print("Received new goal pose")
		goal_loc = location_from_pose(msg.pose)
		print(goal_loc)

		self.t = np.array([])
		self.x = np.array([])
		self.y = np.array([])
		self.th = np.array([])
		self.w_left = np.array([])
		self.w_right = np.array([])
		rospy.loginfo('Cleared Current Path')

		res = solve_traj_opt(
			self.robot_loc, goal_loc, wheelradius, w_max, wheelbase
		)
		rospy.loginfo('Optimized new path')

		N, T, dt, self.x, self.y, self.th, self.w_left, self.w_right = res
		time_seconds = rospy.get_time()
		self.t = time_seconds + (np.arange(N) * T)

		p = Path()
		p.header.stamp = msg.header.stamp
		p.header.frame_id = "world"
		for i in range(N):
			path_pose = PoseStamped()
			path_pose.pose.position.x = float(self.x[i])
			path_pose.pose.position.y = float(self.y[i])
			path_pose.pose.position.z = 0
			quat = tf.transformations.quaternion_from_euler(0,0,float(self.th[i]))
			path_pose.pose.orientation.x = quat[0]
			path_pose.pose.orientation.y = quat[1]
			path_pose.pose.orientation.z = quat[2]
			path_pose.pose.orientation.w = quat[3]
			p.poses.append(path_pose)

		self.path_publisher.publish(p)
		rospy.loginfo("Sending out new path")
		self.start_time = rospy.get_time()

	def pose_est_callback(self, msg):
		x, y, th, v, omega = location_and_vel_from_odometry(msg)
		self.robot_loc = [x, y, th]
		self.v = v
		self.omega = omega

def setup_odrive():
    rospy.loginfo("Finding odrive...")
    odrv0 = odrive.find_any()
    rospy.loginfo("Odrive found.")
    return odrv0

def main():
	rospy.init_node('controller')
	
	odrv0 = setup_odrive()

	node = ControllerNode(odrv0)
	rospy.Subscriber('/ui/goal_pose', PoseStamped, node.goal_pose_callback)
	rospy.Subscriber('/localizer/pose_est', Odometry, node.pose_est_callback)
	joint_pub = rospy.Publisher('/state_estimator/joint_states', JointState, queue_size=1)
	#rospy.wait_for_message('/ui/goal_pose', PoseStamped)
	# TODO: uncomment next line once there is localization data
	# rospy.wait_for_message('/localizer/pose_est', Odometry)

	p_cross_track = 1
	p_heading = 2
	p_vel = .5
	delay = 0
	rat = rospy.Rate(250)
	while not rospy.is_shutdown():

		# Collect and send out encoder positions and velocities

		wheel_l_theta = odrv0.axis1.encoder.pos_estimate*(2*np.pi/4000) % 2*np.pi # convert ticks to radians
		wheel_r_theta = -odrv0.axis0.encoder.pos_estimate*(2*np.pi/4000) % 2*np.pi

		wheel_l_dtheta = odrv0.axis1.encoder.vel_estimate*(2*np.pi/4000)
		wheel_r_dtheta = -odrv0.axis0.encoder.vel_estimate*(2*np.pi/4000)

		#rospy.loginfo('w_right_act: {}, w_left_act: {}'.format(wheel_l_dtheta, wheel_r_dtheta))

		v = (wheel_l_dtheta + wheel_r_dtheta)/2*wheelradius
		#rospy.loginfo('v: {}'.format(v))

		js = JointState()
		js.header.stamp = rospy.get_rostime()
		js.name = ["wheel_l", "wheel_r"]
		js.position = [wheel_l_theta, wheel_r_theta]
		js.velocity = [wheel_l_dtheta, wheel_r_dtheta]
		js.effort = [0,0]
		joint_pub.publish(js)

		# Control loop
		if node.t.shape[0] != 0:
			time_seconds = rospy.get_time()
			index = np.searchsorted(node.t, time_seconds)
			rospy.loginfo("Index {}".format(index))
			
			if index >= node.t.shape[0]:
				rospy.loginfo("Reached end of path, stopping")
				# when reached the end of the path just use last point
				index = node.t.shape[0] - 1
				w_right_cmd = 0
				w_left_cmd = 0
			else:
				t_next = node.t[index] - time_seconds
				t_prev = time_seconds - node.t[index - 1]
				r = t_prev / (t_prev + t_next) # ratio for interpolation

				x_des = node.x[index - 1] * (1 - r) + node.x[index] * r
				y_des = node.y[index - 1] * (1 - r) + node.y[index] * r
				th_des = node.th[index - 1] * (1 - r) + node.th[index] * r
				w_left_des = node.w_left[index - 1] * (1 - r) + node.w_left[index] * r
				w_right_des = node.w_right[index - 1] * (1 - r) + node.w_right[index] * r

				v = node.v
				omega = node.omega
				x_est = (node.robot_loc[0] + delay * v * np.cos(node.robot_loc[2]))
				y_est = (node.robot_loc[1] + delay * v * np.sin(node.robot_loc[2]))
				th_est = node.robot_loc[2] + omega * delay

				dx = x_des - x_est
				dy = y_des - y_est
				lookahead_dist = np.sqrt(dx * dx + dy * dy)
				lookahead_theta = np.arctan2(dy, dx)
				
				angle_cross_track = angles.shortest_angular_distance(
					th_est + np.pi * (v < 0), lookahead_theta
				)
				omega_cross_track = lookahead_dist * np.sin(angle_cross_track) * np.sign(v)

				theta_diff = angles.shortest_angular_distance(th_est, th_des)
				omega_heading = theta_diff * np.sign(v)

				# angular velocity command
				omega_des = (wheelradius / wheelbase) * (w_right_des - w_left_des)
				omega_cmd = (
					omega_des + 
					(p_cross_track * omega_cross_track) + 
					(p_heading * omega_cross_track)
				)
				# velocity command
				v_des = wheelradius * (w_left_des + w_right_des) / 2
				v_cmd = v_des + p_vel * (lookahead_dist * np.cos(angle_cross_track) * np.sign(v))
				# wheel angular velocity commands
				w_right_cmd = (v_cmd + omega_cmd * (wheelbase / 2)) / wheelradius
				w_left_cmd = (v_cmd - omega_cmd * (wheelbase / 2)) / wheelradius
				w_right_cmd = np.clip(w_right_cmd, -w_max, w_max)
				w_left_cmd = np.clip(w_left_cmd, -w_max, w_max)
		else:
			w_right_cmd = 0
			w_left_cmd = 0

		# TODO: do something with the desired w_left, w_right here
		#rospy.loginfo('w_right_cmd: {}, w_left_cmd: {}'.format(w_right_cmd, w_left_cmd))

		# Command wheel velocity to odrive

		w_right_ticks_per_sec = w_right_cmd*4000/(2*np.pi)
		w_left_ticks_per_sec = w_left_cmd*4000/(2*np.pi)
		
		odrv0.axis1.controller.vel_setpoint = w_left_ticks_per_sec
		odrv0.axis0.controller.vel_setpoint = -w_right_ticks_per_sec

		rat.sleep()

	#odrv0.axis0.requested_state = AXIS_STATE_IDLE
	#odrv0.axis1.requested_state = AXIS_STATE_IDLE

if __name__ == '__main__':
	main()