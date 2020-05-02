#!/usr/bin/env python
from traj_opt_casadi import solve_traj_opt

#start = [-5.22054303,  2.49322135, -0.66125525, -4.99999999]
#finish = [-2.41064465, -1.4239602 , -0.69652365]
start = [0,0,0]
finish = [-5,5,0]
N,T,dt,xout,yout,thetaout,wleftout,wrightout = solve_traj_opt(start,finish,100)

# ---- post-processing        ------
from pylab import plot, step, figure, legend, show, spy

figure(1)
plot(xout,label="x")
plot(yout,label="y")
plot(thetaout,label="theta")

step(range(N),wleftout,'k',label="w left")
step(range(N),wrightout,'k',label="w right")
legend(loc="upper left")

figure(2)
plot(xout,yout)

show()