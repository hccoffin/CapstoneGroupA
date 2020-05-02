#!/usr/bin/env python

import numpy as np
from casadi import *



wheelradius = 0.1 # Wheel radius in m
V_max = 0.5 # Maximum forward velocity in m
w_max = V_max/wheelradius

wheelbase = 0.2 # Distance between wheels m
lin_tol = 0.03 #m
ang_tol = 1*np.pi/180.0 #rads

def solve_traj_opt(ics, fcs, N=100):
    opti = Opti() # Optimization problem

    # ---- decision variables ---------
    X = opti.variable(3,N+1) # state trajectory
    x = X[0,:]
    y = X[1,:]
    theta = X[2,:]

    U = opti.variable(2,N)   # control trajectory (throttle)
    w_left = U[0,:]
    w_right = U[1,:]
    T = opti.variable()      # final time

    # ---- objective          ---------
    opti.minimize(T) # race in minimal time

    # ---- dynamic constraints --------
    f = lambda x,u: vertcat(wheelradius/2*cos(x[2])*(u[0] + u[1]), # dx/dt = f(x,u)
                            wheelradius/2*sin(x[2])*(u[0] + u[1]),
                            wheelradius/wheelbase*(u[1] - u[0])) 

    dt = T/N # length of a control interval
    for k in range(N): # loop over control intervals
       # Runge-Kutta 4 integration
       k1 = f(X[:,k],         U[:,k])
       k2 = f(X[:,k]+dt/2*k1, U[:,k])
       k3 = f(X[:,k]+dt/2*k2, U[:,k])
       k4 = f(X[:,k]+dt*k3,   U[:,k])
       x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
       opti.subject_to(X[:,k+1]==x_next) # close the gaps

    # ---- path constraints -----------
    opti.subject_to(opti.bounded(-15,x,15)) # x
    opti.subject_to(opti.bounded(-15,y,15)) # y

    opti.subject_to(opti.bounded(-w_max,w_left,w_max)) # left wheel velocity
    opti.subject_to(opti.bounded(-w_max,w_right,w_max)) # left wheel velocity

    # ---- boundary conditions --------
    opti.subject_to(x[0]==ics[0])   # x0
    opti.subject_to(y[0]==ics[1])   # y0
    opti.subject_to(theta[0]==ics[2])   # theta0
    opti.subject_to(w_left[0]==0) # .v0
    opti.subject_to(w_right[0]==0) # .v0

    opti.subject_to(opti.bounded(fcs[0] - lin_tol,x[-1],fcs[0] + lin_tol))  # xf
    opti.subject_to(opti.bounded(fcs[1] - lin_tol,y[-1],fcs[1] + lin_tol))  # yf
    opti.subject_to(opti.bounded(fcs[2] - ang_tol,theta[-1],fcs[2] + ang_tol))  # thetaf
    opti.subject_to(w_left[-1]==0) # .v0
    opti.subject_to(w_right[-1]==0) # .v0


    # ---- misc. constraints  ----------
    opti.subject_to(T>=0.1) # Time must be positive

    # ---- initial values for solver ---
    x0 = linspace(ics[0], fcs[0], N+1) + 0.05*np.random.rand(N+1,1)
    opti.set_initial(x, x0)
    y0 = linspace(ics[1], fcs[1], N+1) + 0.05*np.random.rand(N+1,1)
    opti.set_initial(y, y0)
    
    opti.set_initial(T,5)

    # ---- solve NLP              ------
    opti.solver("ipopt") # set numerical backend
    try:
        sol = opti.solve()   # actual solve
    except:
        opti.debug.show_infeasibilities()

    res = (N,sol.value(T),sol.value(dt),sol.value(x), sol.value(y), sol.value(theta), sol.value(w_left), sol.value(w_right))
    return res