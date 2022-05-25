#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Anna Astolfi
import numpy as np
from robot_sense import RobotSensing
import rospy
import signal
import sys
import time
import threading
import rosservice
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse
from silver3.srv import BhvSelector, BhvSelectorResponse
from std_msgs.msg import Float32MultiArray


class PoseSensingRos(RobotSensing):

    def __init__(self, ctrl_timestep = 0.01):
        super().__init__()
        print(self.inFnct)
        #ROS services
        # state variables

        # loop parameters
        self.ctrl_timestep = ctrl_timestep #s
        
        # ROS service init

        # ROS publishers
        self.ee_pose_pub = rospy.Publisher('robot_sensing/ee_pose', Float32MultiArray, queue_size= 10)
        rospy.Timer(rospy.Duration(self.ctrl_timestep), self.publish_ee_pose)
        self.ee_pose = Float32MultiArray()
    
    
    def loop(self):
        rosthread = threading.Thread(target=self.ros_spin)
        rosthread.deamon = True
        rosthread.start()
        
        while not rospy.is_shutdown():
            time.sleep(self.ctrl_timestep)

    
    def ros_spin(self):
        rospy.spin()    
    
    def publish_ee_pose(self, event=None):
        try:
            self.ee_pose.data = list(self.read_ee_pose())#.insert(0, rospy.get_time())
        except:
            self.ee_pose.data = [0]*6
        
        self.ee_pose.data.insert(0, rospy.get_time())
        self.ee_pose_pub.publish(self.ee_pose)


def handler(signum, frame):
    global should_quit
    should_quit = True
    
    
if __name__ == '__main__':
    
    rospy.init_node('ee_pose_node')

    # prevhand is the handler set by the ROS runtime
    prevhand = signal.signal(signal.SIGINT, handler)
    pose_sense = PoseSensingRos()

    pose_sense.loop()

    # calls ROS signal handler manually
    # this is to stop the rospy.spin thread
    prevhand(signal.SIGINT, None) 
