#!/usr/bin/env python3

# General Imports
import time
import datetime

# ROS Imports
import rospy
import std_msgs

# Optimization/Control Imports
from cvxopt import matrix, spmatrix, solvers
import numpy as np
from numpy.matlib import repmat
from scipy import linalg

# IO imports
import odrive
import bios # Load dynamics from yaml

# Load A,B dynamics matrices, costs Q, R and LQR backup gains K from YAML
params_filename = "../config/control_params.yaml"
control_params = bios.read(params_filename)

Ad = matrix(np.array(control_params['Ad']))
Bd = matrix(np.array(control_params['Bd']))
Q = matrix(np.array(control_params['Q']))
R = matrix(np.array(control_params['R']))

N = int(control_params['N'])
N = 100
dt = control_params['dt']
nx,nu = Bd.size

# Setup the QP
xr = np.zeros([(N+1)*nx,1])

x0 = np.zeros([nx,1])
xmin = np.reshape(np.array([-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf]), [nx,1])
xmax = np.reshape(np.array([ np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]), [nx,1])
umin = np.reshape(np.array([-np.inf,-np.inf]), [nu,1])
umax = np.reshape(np.array([ np.inf, np.inf]), [nu,1])

# Cost Function
Px = np.kron(np.eye(N+1,N+1), Q)
Pu = np.kron(np.eye(N,N),R)
P = matrix(linalg.block_diag(Px, Pu))

qx = np.matmul(np.kron(np.eye(N+1,N+1), -Q.T), xr)
qu = np.zeros([N*nu, 1])
q = matrix(np.vstack([qx, qu]))

# Bounds Constraints
G_xmin = np.hstack([np.zeros([N*nx,nx]), -np.eye(N*nx,N*nx)])
G_xmax = np.hstack([np.zeros([N*nx,nx]),  np.eye(N*nx,N*nx)])

G_umin = -np.eye(N*nu,N*nu)
G_umax =  np.eye(N*nu,N*nu)

G = matrix(np.vstack([linalg.block_diag(G_xmin, G_umin), linalg.block_diag(G_xmax, G_umax)]))
h = matrix(np.vstack([np.matlib.repmat(-xmin, N, 1),np.matlib.repmat(-umin, N, 1),
					 np.matlib.repmat(xmax, N, 1),np.matlib.repmat(umax, N, 1)]))

# Initial/Final Condition Constraints
A_ic = np.hstack([np.eye(nx,nx), np.zeros([nx, N*nx + N*nu])])
b_ic = x0

# Dynamics Constraints
A_dyn_Ad = np.hstack([np.kron(np.eye(N,N), Ad), np.zeros([N*nx, nx])])
A_dyn_xp = np.hstack([np.zeros([N*nx, nx]), np.kron(np.eye(N,N), np.eye(nx,nx))])
A_dyn_Bd = np.kron(np.eye(N,N), Bd)

A_dyn = np.hstack([A_dyn_Ad - A_dyn_xp, A_dyn_Bd])

b_dyn = np.zeros([N*nx, 1])

A = matrix(np.vstack([A_ic, A_dyn]))
b = matrix(np.vstack([b_ic, b_dyn]))

G = matrix(np.zeros([(N+1)*nx + N*nu,(N+1)*nx + N*nu]))
h = matrix(np.zeros([(N+1)*nx + N*nu,1]))

# Solve the QP
t_start = datetime.datetime.now()
#x_guess = np.zeros([(N+1)*nx + N*nu, 1])
sol=solvers.qp(P, q, G, h, A, b)
t_end = datetime.datetime.now()
t_elapsed = t_end - t_start

# Print solutions and elapsed time
#print(sol['x'])
print("Time elapsed: %.3f ms" % (t_elapsed.microseconds/1000 + t_elapsed.seconds*1000))




