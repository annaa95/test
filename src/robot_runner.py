#!/usr/bin/env python
from curses.ascii import ctrl
import numpy as np
import rospy
import signal
import sys
import time
import threading
#import roslaunch
import rosservice
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse
from silver3.srv import BhvSelector, BhvSelectorResponse
from std_msgs.msg import Float32MultiArray

# import robot_perceiving
from act.robot_actions_ros import RobotActionRos
#from perceive.robot_perception_ros import RobotPerceptionRos
# import robot_thinking

#define constants 

class Runner(object):
    def __init__(self):
        """
        Initialize the robot/simulated robot
        """
        #self.launch_action_nodes()
        #self.launch_perception_nodes()
        #self.launch_planning_nodes()
        self.actions = RobotActionRos(ctrl_timestep = 0.01) #qui saranno inizializzati i servizi/topic publisher...
        #self.perception = RobotPerceptionRos(ctrl_timestep = 0.01) #qui saranno inizializzati i servizi/topic publisher...
        #self.thoughts = RobotPlanning()#qui saranno inizializzati i servizi/topic publisher...
    
    def launch_action_nodes(self):    
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/anna/catkin_ws/src/silver3/launch/silver3_motion_control.launch"])
        self.launch.start()
        rospy.loginfo("started")
    
    def run(self):
        self.actions.loop()

def handler(signum, frame):
    global should_quit
    should_quit = True

if __name__ == '__main__':
    rospy.init_node('robot')

    # prevhand is the handler set by the ROS runtime
    prevhand = signal.signal(signal.SIGINT, handler)
    controller = Runner()

    controller.run()

    # calls ROS signal handler manually
    # this is to stop the rospy.spin thread
    prevhand(signal.SIGINT, None)        






