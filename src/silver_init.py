#!/usr/bin/env python
from RPIO import PWM
import time

azimuth_cam = 18 #Azimuth
elevation_cam = 27 #Elevation
torches = 4 #Torches

servo = PWM.Servo()

pw_elevation = 2000
pw_azimuth = 1650
pw_torches = 0

servo.set_servo(elevation_cam, pw_elevation)
servo.set_servo(azimuth_cam, pw_azimuth)
servo.set_servo(torches, pw_torches)

time.sleep(0.5)
