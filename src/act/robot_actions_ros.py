#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Anna Astolfi
import numpy as np
import sys

#sys.path.insert(1, '/home/anna/catkin_ws/src/silver3/src/act')

from .robot_actions import RobotAction
import rospy
import signal
import time
import threading
import rosservice
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse
from silver3.srv import BhvSelector, BhvSelectorResponse
from std_msgs.msg import Float32MultiArray


class RobotActionRos(RobotAction):

    def __init__(self, ctrl_timestep = 0.01):
        super().__init__()
        #ROS services
        # state variables
        self.should_quit = False
        self.bhv_running = False
        self.bhv_type = None
        self.bhv_start = False

        # loop parameters
        self.ctrl_timestep = ctrl_timestep #s
        
        #intialize the node 
        #rospy.init_node('motion_control_node')

        # ROS service init
        self.set_bhv_proxy = rospy.Service('robot_actions/set_bhv', BhvSelector, self.set_bhv)
        self.start_bhv_proxy = rospy.Service('robot_actions/start_bhv', Empty, self.start_bhv)
        self.stop_bhv_proxy = rospy.Service('robot_actions/stop_bhv', Empty, self.stop_bhv)
        
        # ROS publishers
        self.motor_status_pub= rospy.Publisher('robot_action/motor_status', Float32MultiArray, queue_size=10)
        rospy.Timer(rospy.Duration(self.ctrl_timestep), self.publish_motor_status)
        self.motor_status = Float32MultiArray()
    
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
    
    def stop_bhv(self, req):
        self.bhv_running = False
        self.ctrl_timestep = 0.01
        print('Behavior stopped')
	    #self.robot.change_configuration(Q_idle)
        self.bhv_type = None
        return EmptyResponse()

    def set_bhv(self, req):

        if self.bhv_running:
            return BhvSelectorResponse(False, 'cannot set bhv while moving')

        if req.type == 'demo':
            self.bhv_type = 'demo'
            self.bhv_params = {
                'nstep': req.nstep,
                }
            # check params
            if self.bhv_params['nstep'] <= 0:
                self.bhv_params['nstep'] = 120

            # compute trajectory, check feasibility and apply inverse kinematics
            feasibility = self.generate_trj(int(self.bhv_params['nstep']))
            if feasibility:
                # goto initial position
                self.goto_pose(self.Qdot[:,0],self.Q[:,0])
                return BhvSelectorResponse(True, 'Demo successfully set.')
            else:
                self.bhv_type = None
                return BhvSelectorResponse(False, 'Failed to set demo. Non feasible EndEff trajectory.')

        else:
            self.bhv_running = False
            self.bhv_type = None
            self.ctrl_timestep = 0.01
            return BhvSelectorResponse(False, 'Failed to set Behavior. Invalid bhv type selected.')
    
    
    def start_bhv(self, req):
            
        if self.bhv_type is not None:
            if self.bhv_type == "demo":
                print("starting demo")
            elif self.bhv_type == "demo2":
                print("starting demo2")
            else:
                print("unknown bhv")
        else:
            print("Bhv is None")

        self.bhv_running = True
        return EmptyResponse()

    def publish_motor_status(self, event=None):
        self.motor_status.data = self.read_encoders()#.insert(0, rospy.get_time())
        self.motor_status.data.insert(0, rospy.get_time())
        self.motor_status_pub.publish(self.motor_status) 


def handler(signum, frame):
    global should_quit
    should_quit = True
    
    
if __name__ == '__main__':
    
    rospy.init_node('motion_control_node')

    # prevhand is the handler set by the ROS runtime
    prevhand = signal.signal(signal.SIGINT, handler)
    controller = RobotActionRos()

    controller.loop()

    # calls ROS signal handler manually
    # this is to stop the rospy.spin thread
    prevhand(signal.SIGINT, None) 