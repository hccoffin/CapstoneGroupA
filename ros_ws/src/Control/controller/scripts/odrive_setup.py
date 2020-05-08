#!/usr/bin/env python3
import odrive
from odrive.enums import *
from odrive.utils import *
import time

print ("Looking for odrive...")
odrv0 = odrive.find_any()
print ("Found odrive!")

odrv0.erase_configuration()
dump_errors(odrv0,True)
print(" ")
dump_errors(odrv0)

for axis in ['axis0']:
    print("Configuring {}...".format(axis))

    print("     Configuring motor...")
    getattr(odrv0, axis).motor.config.current_lim_tolerance = 3.0
    getattr(odrv0, axis).motor.config.pole_pairs = 21
    getattr(odrv0, axis).encoder.config.use_index = True
    getattr(odrv0, axis).requested_state = AXIS_STATE_MOTOR_CALIBRATION
    while getattr(odrv0, axis).current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    getattr(odrv0, axis).motor.config.pre_calibrated = True
    print("     Motor configured.")

    print("     Configuring encoder...")
    getattr(odrv0, axis).encoder.config.cpr = 4000
    getattr(odrv0, axis).requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
    while getattr(odrv0, axis).current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    getattr(odrv0, axis).requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    while getattr(odrv0, axis).current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    getattr(odrv0, axis).encoder.config.pre_calibrated = True

    print("     Start full calibration sequence...")
    getattr(odrv0, axis).requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while getattr(odrv0, axis).current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    print("     Calibrated.")

print(" ")
dump_errors(odrv0)
odrv0.save_configuration()
