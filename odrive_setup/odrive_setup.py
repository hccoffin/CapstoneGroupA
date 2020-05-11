#!/usr/bin/env python3
import odrive
from odrive.enums import *
from odrive.utils import *
import time

print ("Looking for odrive...")
odrv0 = odrive.find_any()
print ("Found odrive!")

dump_errors(odrv0)

for axis in ['axis0', 'axis1']:
    print("Configuring motor...")
    getattr(odrv0, axis).motor.config.resistance_calib_max_voltage = 3
    getattr(odrv0, axis).motor.config.current_lim_tolerance = 3
    getattr(odrv0, axis).motor.config.pole_pairs = 21
    getattr(odrv0, axis).controller.config.vel_limit = 20000
    getattr(odrv0, axis).controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    print("Motor configured.")

    print("Configuring encoder...")
    getattr(odrv0, axis).encoder.config.use_index = True
    getattr(odrv0, axis).encoder.config.cpr = 4000
    print("Encoder configured")

    print("Setting startup routine...")
    getattr(odrv0, axis).config.startup_motor_calibration = True
    getattr(odrv0, axis).config.startup_encoder_index_search = True
    getattr(odrv0, axis).config.startup_encoder_offset_calibration = True
    getattr(odrv0, axis).config.startup_closed_loop_control = True
    print("Startup routine set.")


odrv0.save_configuration()
dump_errors(odrv0, True)

