#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Giacomo Picardi

import threading
import rospy
from silver_msgs.srv import CameraAzimuthSet, CameraElevationSet, TorchesIntensitySet, GimbleModeSelection, \
     CameraAzimuthSetResponse, CameraElevationSetResponse, TorchesIntensitySetResponse, GimbleModeSelectionResponse
import math
import pigpio
from sensor_msgs.msg import Imu
import time
import signal

azimuth_pin = 27 #Azimuth
elevation_pin = 18 #Elevation
torches_pin = 4 #Torches

should_quit = False
def handler(signum, frame):
    global should_quit
    should_quit = True


class GimbleController:

    def __init__(self):

        #create pwm controller
        self.pi = pigpio.pi()

        #create services
        self.set_mode_proxy = rospy.Service('camera_gimble/set_mode', GimbleModeSelection, self.set_mode)
        self.set_azimuth_proxy = rospy.Service('camera_gimble/set_azimuth', CameraAzimuthSet, self.set_azimuth)
        self.set_elevation_proxy = rospy.Service('camera_gimble/set_elevation', CameraElevationSet, self.set_elevation)
        self.set_torches_intensity_proxy = rospy.Service('camera_gimble/set_torches_intensity', TorchesIntensitySet, self.set_torches_intensity)

        #subscribe to topics
        self.imu_subscriber = rospy.Subscriber("imu_bosch/data", Imu, self.elevation_compensation)

        #create shared variables
        self.gimble_mode = 'manual'
        self.elevation_offset = 0
        self.ctrl_timestep = 0.01

        #set initial position
        self.pi.set_servo_pulsewidth(azimuth_pin, 1600)
        self.pi.set_servo_pulsewidth(elevation_pin, 2000)
        

    def start(self):
        rosthread = threading.Thread(target=self.ros_spin)
        rosthread.deamon = True
        rosthread.start()
        while True:
            if should_quit:
                break
            time.sleep(self.ctrl_timestep)
        self.pi.stop()
                        
    def ros_spin(self):
        rospy.spin()

    def set_mode(self, req):
        if req.mode == 'manual':
            self.gimble_mode = 'manual'
            return GimbleModeSelectionResponse(True, 'Manual mode on')
        elif req.mode == 'auto':
            self.gimble_mode = 'auto'
            return GimbleModeSelectionResponse(True, 'Auto mode on')
        else:
            self.gimble_mode = 'manual'
            return GimbleModeSelectionResponse(False, 'Spelling error - Default manual mode on')

    def set_azimuth(self, req):
        # check req
        print req.azimuth
        pw = req.azimuth
        try:
            self.pi.set_servo_pulsewidth(azimuth_pin, pw)
        except:
            return CameraAzimuthSetResponse(False, 'Failed to set azimuth')
        return CameraAzimuthSetResponse(True, 'Azimuth set correctly')

    def set_elevation(self, req):
        # check req
        print req.elevation
        pw = req.elevation
        try:
            self.pi.set_servo_pulsewidth(elevation_pin, pw)
        except:
            return CameraElevationSetResponse(False, 'Failed to set elevation')
        return CameraElevationSetResponse(True, 'Elevation set correctly')

    def set_torches_intensity(self, req):
        # check req
        print req.intensity
        pw = req.intensity
        try:
            self.pi.set_servo_pulsewidth(torches_pin, pw)
        except:
            return TorchesIntensitySetResponse(False, 'Failed to set intensity')
        return TorchesIntensitySetResponse(True, 'Intensity set correctly')

    def elevation_compensation(self, data):
        if self.gimble_mode == 'auto':
            q1 = data.orientation.x
            q2 = data.orientation.y
            q3 = data.orientation.z
            q0 = data.orientation.w
            q_norm = math.sqrt(q0*q0+q1*q1+q2*q2+q3*q3)
            q1 = data.orientation.x/q_norm
            q2 = data.orientation.y/q_norm
            q3 = data.orientation.z/q_norm
            q0 = data.orientation.w/q_norm
        
            phi = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
            theta = math.asin(2*(q0*q2-q3*q1))
            psi = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))

            #rospy.loginfo(rospy.get_caller_id() + "bank: %.2f, attitude: %.2f, heading: %.2f", phi*180/math.pi, theta*180/math.pi, psi*180/math.pi)
        
            pw = round(theta/math.pi*2*180)*10+1800
            #self.pwm_controller.set_servo(elevation_pin, pw)

if __name__ == '__main__':

    rospy.init_node('gimbal_node')

    # prevhand is the handler set by the ROS runtime
    prevhand = signal.signal(signal.SIGINT, handler)

    controller = GimbleController()

    controller.start()

    # calls ROS signal handler manually
    # this is to stop the rospy.spin thread
    prevhand(signal.SIGINT, None)
