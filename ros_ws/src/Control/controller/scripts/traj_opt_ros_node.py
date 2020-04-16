#!/usr/bin/env python

import rospy
from common import *
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Twist
from ackermann_msgs.msg import AckermannDriveStamped
from angles import *
import tf
import time
import numpy as np

from traj_opt_casadi import solve_traj_opt

# Configurable params
show_output = False
plot_result = True

def waypointCallback(msg):
  global waypoints
  for i in range(len(msg.poses)):
    waypoints[i, 0] = msg.poses[i].position.x
    waypoints[i, 1] = msg.poses[i].position.y
    waypoints[i, 2] = euler_from_quaternion([msg.poses[i].orientation.x, msg.poses[i].orientation.y, msg.poses[i].orientation.z, msg.poses[i].orientation.w])[2]

def vehicleStateCallback(msg):
  global cur_x, cur_y, cur_theta, cur_v_lin, cur_v_ang
  cur_x = msg.pose.pose.position.x
  cur_y = msg.pose.pose.position.y
  cur_orientation = msg.pose.pose.orientation

  cur_theta = euler_from_quaternion(
    [cur_orientation.x, cur_orientation.y, cur_orientation.z,
     cur_orientation.w])[2]

  cur_v_lin = (
    .5 * msg.twist.twist.linear.x / (np.cos(cur_theta) + 1e-5) + 
    .5 * msg.twist.twist.linear.y / (np.sin(cur_theta) + 1e-5)
  )
  cur_v_ang = msg.twist.twist.angular.z

if __name__ == '__main__':

  rospy.init_node('controller_node')
  cmd_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=10)
  path_pub = rospy.Publisher('/ackermann_vehicle/desired_path', Path, queue_size=10)

  waypoints = np.zeros((num_waypoints, 3))
  rospy.Subscriber("/ackermann_vehicle/waypoints",
                   PoseArray,
                   waypointCallback)
  rospy.wait_for_message("/ackermann_vehicle/waypoints", PoseArray, 5)

  rear_axle_center = Pose()
  rear_axle_velocity = Twist()
  rospy.Subscriber("/ackermann_vehicle/ground_truth/state",
                   Odometry, vehicleStateCallback)
  rospy.wait_for_message("/ackermann_vehicle/ground_truth/state", Odometry, 5)
  time.sleep(0.01)

  # Generate path
  x_accum = np.array([])
  y_accum = np.array([])
  theta_accum = np.array([])
  v_accum = np.array([])
  a_accum = np.array([])
  alpha_accum = np.array([])
  dts = np.array([])
  previous = np.array([cur_x, cur_y, cur_theta, 0])
  for i in range(0, num_waypoints):
    target = waypoints[i,:]

    # Construct and solve NLP
    print("Previous: ", previous)
    print("Target: ", target)
    rospy.loginfo("Solving traj opt for subpath %d" % i)
    res = solve_traj_opt(previous,target)
    rospy.loginfo("Solved.")
    N,T,dt,xout,yout,thetaout,vout,aout,alphaout = res
    
    # Crop off first state from each state variable
    xout = xout[1:]
    yout = yout[1:]
    thetaout = thetaout[1:]

    achieved = [xout[-1],yout[-1],thetaout[-1],vout[-1]]

    x_accum = np.concatenate((x_accum, xout), axis=None)
    y_accum = np.concatenate((y_accum, yout), axis=None)
    theta_accum = np.concatenate((theta_accum, thetaout), axis=None)
    v_accum = np.concatenate((v_accum, vout), axis=None)
    a_accum = np.concatenate((a_accum, aout), axis=None)
    alpha_accum = np.concatenate((alpha_accum, alphaout), axis=None)
    dts = np.concatenate((dts, np.ones(xout.shape[0]) * dt))

    # As we increment target becomes previous
    previous = np.append(target, vout[-1])

  path = Path()
  path.header.frame_id = "map"
  T_accum = np.array([])
  T_cur = 0
  for w in range(0,x_accum.size):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.from_sec(T_cur)
    pose.header.frame_id = "map"
    pose.pose.position.x = x_accum[w]
    pose.pose.position.y = y_accum[w]
    q = quaternion_from_euler(0, 0, theta_accum[w])
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    path.poses.append(pose)
    
    T_cur += dts[w]
    T_accum = np.append(T_accum, T_cur)

  path_pub.publish(path) # Can visualize this in RVIZ to check

  # LOCAL CONTROLLER GOES HERE
  t_start = rospy.get_time()
  t = 0
  cmd = AckermannDriveStamped()
  cmd.header.frame_id = "base_link"
  p_xy = 1
  p_heading = 2
  p_vel = .5
  delay = 0.28

  while t < T_accum[-1]:
    cmd.header.stamp = rospy.Time.now()
    index = np.searchsorted(T_accum, t)
    t_next = T_accum[index] - t
    t_prev = t - T_accum[index - 1]
    interp_ratio = t_prev / (t_prev + t_next)

    x_des = x_accum[index - 1] * (1 - interp_ratio) + x_accum[index] * interp_ratio
    y_des = y_accum[index - 1] * (1 - interp_ratio) + y_accum[index] * interp_ratio
    theta_des = theta_accum[index - 1] * (1 - interp_ratio) + theta_accum[index] * interp_ratio
    v_des = v_accum[index - 1] * (1 - interp_ratio) + v_accum[index] * interp_ratio
    alpha_des = alpha_accum[index - 1] * (1 - interp_ratio) + alpha_accum[index] * interp_ratio

    x = (cur_x + delay * cur_v_lin * np.cos(cur_theta))
    y = (cur_y + delay * cur_v_lin * np.sin(cur_theta))
    theta = cur_theta + cur_v_ang * delay

    dx = x_des - x
    dy = y_des - y
    lookahead_dist = np.sqrt(dx * dx + dy * dy)
    lookahead_theta = np.arctan2(dy, dx)
    
    angle_xy = shortest_angular_distance(theta + np.pi * (cur_v_lin < 0), lookahead_theta)
    alpha_xy = lookahead_dist * np.sin(angle_xy) * np.sign(cur_v_lin)

    theta_diff = shortest_angular_distance(theta, theta_des)
    alpha_heading = theta_diff * np.sign(cur_v_lin)

    alpha_cmd = alpha_des + (p_xy * alpha_xy) + (p_heading * alpha_heading)
    alpha_cmd = np.clip(alpha_cmd, -30 * np.pi / 180, 30 * np.pi / 180)

    v_cmd = v_des + p_vel * lookahead_dist * np.cos(angle_xy) * np.sign(cur_v_lin)
    v_cmd = np.clip(v_cmd, -1.2, 1.2)

    cmd.drive.speed = v_cmd
    cmd.drive.acceleration = 1
    cmd.drive.steering_angle = alpha_cmd

    cmd_pub.publish(cmd)
    rospy.wait_for_message("/ackermann_vehicle/ground_truth/state", Odometry, 5)
    t = rospy.get_time() - t_start

    print(theta_diff, lookahead_dist * np.cos(angle_xy) * np.sign(cur_v_lin))