#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Anna Astolfi
import numpy as np
from .robot_perception import RobotPerception
import rospy
import signal
import sys
import time
import threading
import rosservice
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse
from silver3.srv import BhvSelector, BhvSelectorResponse
from std_msgs.msg import Float32MultiArray


class RobotPerceptionRos(RobotPerception):

    def __init__(self, ctrl_timestep = 0.01):
        
        super(RobotPerception, self).__init__()
        #ROS services
        # state variables
        self.should_quit = False
        self.bhv_running = False
        self.bhv_type = None
        self.bhv_start = False

        # loop parameters
        self.ctrl_timestep = ctrl_timestep #s
        
        # ROS service init
        self.set_bhv_proxy = rospy.Service('robot_actions/set_bhv', BhvSelector, self.set_bhv)
        self.start_bhv_proxy = rospy.Service('robot_actions/start_bhv', Empty, self.start_bhv)
        self.stop_bhv_proxy = rospy.Service('robot_actions/stop_bhv', Empty, self.stop_bhv)
        
        # ROS publishers
        self.accelerometer_pub = rospy.Publisher('robot_perception/accel_status', Float32MultiArray, queue_size= 10)
        rospy.Timer(rospy.Duration(self.ctrl_timestep), self.publish_accelerometer_status)
    
    def loop(self):
        rosthread = threading.Thread(target=self.ros_spin)
        rosthread.deamon = True
        rosthread.start()
        
        while True:

            if self.should_quit:
                break

            if self.bhv_running:
                if self.bhv_type== "demo":
                    """
                    The manipulator span the workpace of all joints sequentially
                    """ 
                    if self.demo():
                        continue
                    else:
                        self.bhv_running = False
                elif self.bhv_type == "demo2":
                    """
                    other stuff
                    """
                else:
                    print('unknown behavior')
            
            time.sleep(self.ctrl_timestep)

        if self.bhv_running:
            self.stop_bhv(None)

        self.__del__()
    
    def ros_spin(self):
        rospy.spin()    
    
    def publish_accelerometer_status(self, event=None):
        self.motor_status.data = self.perceive.read_accelerometer()#.insert(0, rospy.get_time())
        self.motor_status.data.insert(0, rospy.get_time())
        self.motor_status_pub.publish(self.motor_status)


def handler(signum, frame):
    global should_quit
    should_quit = True
    
    
if __name__ == '__main__':
    
    rospy.init_node('uw_gripper')

    # prevhand is the handler set by the ROS runtime
    prevhand = signal.signal(signal.SIGINT, handler)
    controller = RobotActionRos()

    controller.loop()

    # calls ROS signal handler manually
    # this is to stop the rospy.spin thread
    prevhand(signal.SIGINT, None) 