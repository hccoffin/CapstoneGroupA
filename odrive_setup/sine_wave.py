import odrive
from odrive.enums import *

from matplotlib import pyplot as plt
import numpy as np
import time

odrv0 = odrive.find_any()

print("Current Voltage: %0.3f" % odrv0.vbus_voltage)

odrv0.axis0.controller.config.vel_limit = 30000
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH

while odrv0.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
print("Completed encoder index search")

odrv0.axis0.controller.current_setpoint = 0
odrv0.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

print("Starting forcing function sweep")

# Setup forcing function
tf = 30
dt = 0.005

omega = 6
mag = 5

ts = np.arange(0,tf,dt)

xs = np.array([])
vs = np.array([])
Is_measured = np.array([])
Is_commanded = np.array([])

last_time = time.time()
for i in range(0, ts.size):
    next_time = last_time + dt
    t = ts[i]
    desired_current = mag*np.sin(omega*t)
    odrv0.axis0.controller.current_setpoint = desired_current

    xs = np.append(xs, odrv0.axis0.encoder.pos_estimate)
    vs = np.append(vs, odrv0.axis0.encoder.vel_estimate)
    Is_measured = np.append(Is_measured, odrv0.axis0.motor.current_control.Iq_measured)
    Is_commanded = np.append(Is_commanded, desired_current)

    sleep_time = next_time - time.time()
    if (sleep_time > 0):
        time.sleep(sleep_time)
    else:
        print(sleep_time)
        print("Time step too short! Exiting now")
        break

    last_time = time.time()

odrv0.axis0.requested_state = AXIS_STATE_IDLE

plt.figure(1)
plt.plot(ts,xs)
plt.title("Position")

plt.figure(2)
plt.plot(ts,vs)
plt.title("Velocity")

plt.figure(3)
plt.plot(ts,Is_commanded)
plt.plot(ts,Is_measured)
plt.title("Current")


plt.show()
