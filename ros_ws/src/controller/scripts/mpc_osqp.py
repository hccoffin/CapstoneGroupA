#!/usr/bin/env python3

# General Imports
import time

# ROS Imports
import rospy
import std_msgs

# Optimization/Control Imports
import osqp
import numpy as np
import scipy as sp
import scipy.io as sio
from scipy import sparse

# MCU imports
import odrive

# Load A,B dynamics matrices, costs Q, R and LQR backup gains K from YAML
params_filename = "../config/control_params.mat"
control_params = sio.loadmat(params_filename)

Ad = control_params['Ad']
Bd = control_params['Bd']
Q = control_params['Q']
R = control_params['R']
dt = control_params['dt']
Qn = 3*Q # Slightly higher weights on final state error
nx,nu = Bd.shape
N = 250 # num states to look ahead

print(control_params)



prob = osqp.OSQP()

prob.solve()
