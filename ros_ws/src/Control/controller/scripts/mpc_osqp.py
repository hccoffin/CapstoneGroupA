#!/usr/bin/env python3

# General Imports
import time

# ROS Imports
import rospy
import std_msgs

# Optimization/Control Imports
import osqp
import numpy as np
from numpy.matlib import repmat
import scipy as sp
from scipy import sparse

# IO imports
import odrive
import bios

# Load A,B dynamics matrices, costs Q, R and LQR backup gains K from YAML
params_filename = "../config/control_params.yaml"
control_params = bios.read(params_filename)

Ad = sparse.csc_matrix(control_params['Ad'])
Bd = sparse.csc_matrix(control_params['Bd'])
Q = sparse.csc_matrix(control_params['Q'])
R = sparse.csc_matrix(control_params['R'])
Qn = 3*Q # Slightly higher weights on final state error
N = int(control_params['N'])
N = 100
dt = control_params['dt']
nx,nu = Bd.shape

# Setup objective (1/2*x'*P*x + q'*x)
xr = np.zeros([nx, 1]) # reference x
x0 = np.reshape(np.array([0.5,0,0,0,0,0]), [nx, 1]) # initial x
P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), Qn, sparse.kron(sparse.eye(N), R)], format='csc')

q = np.vstack([repmat(np.dot(-Q.todense(), xr), N, 1), np.dot(-Qn.todense(), xr), np.zeros([N*nu,1])])

# Setup equality constraints (dynamics)
Adx1 = sparse.kron(sparse.eye(N), Ad)
Adx2 = np.zeros([N*nx, nx])
Adx = sparse.csc_matrix(sparse.hstack([Adx1, Adx2]))
x_plus1 = np.zeros([N*nx,nx])
x_plus2 = sparse.kron(sparse.eye(N), sparse.eye(nx))
x_plus = sparse.csc_matrix(sparse.hstack([x_plus1, x_plus2]))
print(Bd.shape)
Bdu = sparse.kron(sparse.eye(N), Bd)
print(Bdu.shape)

Aeq = sparse.hstack([Adx-x_plus, Bdu])
leq = np.zeros([N*nx, 1])
ueq = np.zeros([N*nx, 1])

# Setup inequality constraints (initial conditions, bounds)
xmin = np.reshape(np.array([-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf]), [nx,1])
xmax = np.reshape(np.array([np.inf, np.inf, np.inf, np.inf, np.inf, np.inf]), [nx, 1])
umin = np.reshape(np.array([-2, -2]), [nu, 1])
umax = np.reshape(np.array([2, 2]), [nu, 1])

Aineq = sparse.eye(nx+N*nx+N*nu) # N+1 constrains on x, N constraints on u
lineq = np.vstack([x0, repmat(xmin, N, 1), repmat(umin, N, 1)])
uineq = np.vstack([x0, repmat(xmax, N, 1), repmat(umax, N, 1)])


# Stack eq and ineq constraints
A = sparse.vstack([Aeq, Aineq], format='csc')
l = np.vstack([leq, lineq])
u = np.vstack([ueq, uineq])

# Setup OSQP instance and solve
prob = osqp.OSQP()

prob.setup(P, q, A, l, u, warm_start=True)

res = prob.solve()
