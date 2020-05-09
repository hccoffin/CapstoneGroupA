#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf

wheelradius = 0.095
wheelbase = 0.2

def print_state_est(msg):
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
	print(np.round([x, y, th, v, omega], decimals=5))

def main():
	rospy.init_node('state_estimator')
	fake_pub = rospy.Publisher(
		'/state_estimator/joint_states', JointState, queue_size=1
	)
	rospy.Subscriber(
		'/localizer/pose_est', Odometry, print_state_est
	)

	r = rospy.Rate(250)
	while not rospy.is_shutdown():
		t_seconds = rospy.get_time();
		js = JointState()
		js.header.stamp = rospy.get_rostime()
		js.name = ["wheel_l", "wheel_r"]
		js.velocity = [.05 / wheelradius, .15 / wheelradius]
		js.effort = [0,0]
		fake_pub.publish(js)
		r.sleep()

if __name__ == '__main__':
	main()