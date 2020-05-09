#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

wheelradius = 0.095
wheelbase = 0.2

class DeadReckoningNode(object):

	def __init__(self, pose_estimate_pub, x_init=0, y_init=0, th_init=0):
		self.pose_estimate_pub = pose_estimate_pub
		self.x = x_init
		self.y = y_init
		self.th = th_init

		self.prev_l_dtheta = 0
		self.prev_r_dtheta = 0
		self.t_prev = 0
		self.seen_msg = False

	def update_estimate(self, msg):
		l_dtheta, r_dtheta = msg.velocity
		t = msg.header.stamp.to_sec()
		if self.seen_msg:
			dt = t - self.t_prev
			l_dtheta_mid = (l_dtheta + self.prev_l_dtheta) / 2
			r_dtheta_mid = (r_dtheta + self.prev_r_dtheta) / 2

			# use the midpoint of the previous velocities and current velocities
			v = wheelradius * (l_dtheta_mid + r_dtheta_mid) / 2
			v_ang = (wheelradius / wheelbase) * (r_dtheta_mid - l_dtheta_mid)

			if v_ang < 1e-5:
				# for better numeric stability when the angular velocity is small
				self.x = self.x + v * np.cos(self.th + v_ang * dt / 2) * dt
				self.y = self.y + v * np.sin(self.th + v_ang * dt / 2) * dt
				self.th = self.th + v_ang * dt
			else:
				dth = v_ang * dt
				self.x = self.x + (v / v_ang) * (np.sin(self.th + dth) - np.sin(self.th))
				self.y = self.y - (v / v_ang) * (np.cos(self.th + dth) - np.cos(self.th))
				self.th = self.th + dth

			# current twists
			vx_cur = np.cos(self.th) * wheelradius * (l_dtheta + r_dtheta) / 2
			vy_cur = np.sin(self.th) * wheelradius * (l_dtheta + r_dtheta) / 2
			dth_cur = (wheelradius / wheelbase) * (r_dtheta - l_dtheta)

			pose_est = Odometry()
			pose_est.header.stamp = msg.header.stamp
			pose_est.header.frame_id = "world"
			pose_est.pose.pose.position.x = self.x
			pose_est.pose.pose.position.y = self.y
			quaternion = quaternion_from_euler(0, 0, self.th)
			pose_est.pose.pose.orientation.x = quaternion[0]
			pose_est.pose.pose.orientation.y = quaternion[1]
			pose_est.pose.pose.orientation.z = quaternion[2]
			pose_est.pose.pose.orientation.w = quaternion[3]
			pose_est.twist.twist.linear.x = vx_cur
			pose_est.twist.twist.linear.y = vy_cur
			pose_est.twist.twist.angular.z = dth_cur
			self.pose_estimate_pub.publish(pose_est)

		self.prev_l_dtheta = l_dtheta
		self.prev_r_dtheta = r_dtheta
		self.t_prev = t
		self.seen_msg = True

def main():
	rospy.init_node('localizer')

	pose_estimate_pub = rospy.Publisher(
		'/localizer/pose_est', Odometry, queue_size=1
	)
	node = DeadReckoningNode(pose_estimate_pub)
	rospy.Subscriber(
		'/state_estimator/joint_states', JointState, node.update_estimate
	)
	rospy.spin()


if __name__ == '__main__':
	main()