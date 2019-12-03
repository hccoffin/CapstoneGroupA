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
from scipy import sparse

# IO imports
import bios

# Load A,B dynamics matrices, costs P, R and LQR backup gains K from YAML
params_filename = "../config/controller.yaml"
control_params = bios.read(params_filename)
print(control_params)

prob = osqp.OSQP()

prob.solve()
