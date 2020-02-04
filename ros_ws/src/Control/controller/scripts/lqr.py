#!/usr/bin/env python

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
import tf

# Linear Algebra imports
import numpy as np

# IO imports
import odrive

class Controller(object):
    def __init__(self, _K, _dt, _sim = True):

        self.tau_min = -5
        self.tau_max = 5
        self.wheel_radius = 0.1
        # Subscribers
        rospy.Subscriber("/planner/current_plan", PlanWithVel,self.planCallback)
        if (_sim):
            self.left_torque_pub = rospy.Publisher("/controller/left_torque_controller/command", Float64, queue_size=1)
            self.right_torque_pub = rospy.Publisher("/controller/right_torque_controller/command", Float64, queue_size=1)
            self.imu_sub = rospy.Subscriber("/controller/imu", Imu, self.imuCallback)
            self.tf_listener_ = tf.TransformListener()

        else:
            self.imu_pub = rospy.Publisher("/controller/imu", Imu, queue_size = 1)

        self.wheel_pub = rospy.Publisher("/controller/wheel_estimate", WheelEst, queue_size = 1)

        self.plan = None
        self.frequency = 1.0/_dt
        self.K = _K
        self.sim = _sim

        self.pitch = 0
        self.dpitch = 0

    def imuCallback(self, imuMsg):
        quat_imu = imuMsg.orientation
        quat_numpy = np.array([quat_imu.w,quat_imu.x,quat_imu.y,quat_imu.z])
        rpy = tf.transformations.euler_from_quaternion(quat_numpy)
        # Looks like yaw is pitch here
        self.pitch = rpy[2]
        self.dpitch = imuMsg.angular_velocity.z


    def planCallback(self, newPlanMsg):
        self.plan = newPlanMsg

    def getMotorCurrent(self, tau_des):
        return np.array([0, 0]).T

    def loop(self):
        rate = rospy.Rate(self.frequency) # ROS sleep Rate
        while not rospy.is_shutdown():
            # Get encoder feedback from ODrive

            # Assemble state estimate and state error
            state_est = np.array([0,self.pitch,0,self.dpitch]).reshape([4,1])
            state_des = np.zeros([4,1])
            if (self.plan == None):
                state_des = np.zeros([4, 1])

            # Convert state error to torques
            state_error = state_des - state_est
            tau_des = -np.matmul(self.K,state_error)
            print(tau_des)
            # Convert F_des to tau1 and tau2
            #tau_des = f_des*self.wheel_radius
            left_torque = tau_des/2.0
            if (left_torque > self.tau_max):
                left_torque = self.tau_max
            elif(left_torque < self.tau_min):
                left_torque = self.tau_min

            right_torque = tau_des/2.0
            if (right_torque > self.tau_max):
                right_torque = self.tau_max
            elif(right_torque < self.tau_min):
                right_torque = self.tau_min


            if (self.sim):
                # Send torques to gazebo
                self.left_torque_pub.publish(-left_torque)
                self.right_torque_pub.publish(-right_torque)
                
            else:
                #Convert desired torques to motors
                current_des = self.getMotorCurrent(tau_des)
                
                #Send out Imu data
                imu_msg = Imu()
                self.imu_pub.publish(imu_msg)

                # Send out new encoder data
                wheel_msg = WheelEst()
                self.wheel_pub.publish(wheel_msg)

            # Enforce constant rate
            rate.sleep()

if __name__ == '__main__':
    # Logging level DEBUG, INFO, WARN, ERROR, FATAL
    rospy.init_node('controller',log_level=rospy.DEBUG)

    time.sleep(1)
    # Load K, dt from ROS parameter server
    K = np.array(rospy.get_param('controller/lqr_gains')).reshape((1,4))
    rospy.loginfo("Loaded Gains: ")
    rospy.loginfo(K)

    dt = rospy.get_param('dt')
    rospy.loginfo("Loaded dt: " + str(dt))

    c = Controller(K, dt)
    c.loop()
