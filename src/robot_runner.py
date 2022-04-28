#!/usr/bin/env python
import numpy as np
import rospy
import signal
import sys
import time
import threading
import rosservice
from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse
#from srv import BhvSelector, BhvSelectorResponse

# import robot_perceiving
from act.robot_actions import RobotAction
# import robot_thinking

#define constants 

class Runner(object):
    def __init__(self):
        """
        Initialize the robot/simulated robot
        """
        self.actions = RobotAction() #qui saranno inizializzati i servizi/topic publisher...
        #self.perception = RobotPerception() #qui saranno inizializzati i servizi/topic publisher...
        #self.thoughts = RobotPlanning()#qui saranno inizializzati i servizi/topic publisher...
        self.should_quit = False
        
        # state variables
        self.bhv_type = None
        self.bhv_start = False

        # loop parameters
        self.ctrl_timestep = 0.01
        
        # ROS service init
        self.set_bhv_proxy = rospy.Service('robot_actions/set_bhv', BhvSelector, self.set_bhv)
        self.start_bhv_proxy = rospy.Service('robot_actions/start_bhv', Empty, self.start_bhv)
        self.stop_bhv_proxy = rospy.Service('robot_actions/stop_bhv', Empty, self.stop_bhv)

    
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
                    self.actions.demo()
                elif self.bhv_type == "demo2":
                    """
                    other stuff
                    """
                else:
                    print('unknown behavior')
            
            time.sleep(self.ctrl_timestep)

        if self.bhv_running:
            self.stop_bhv(None)

        self.actions.__del__()
    
    def stop_bhv(self, req):
        self.bhv_running = False
        self.ctrl_timestep = 0.01
        print('Behavior stopped')
	    #self.robot.change_configuration(Q_idle)
        self.bhv_type = None
        return EmptyResponse()

    def ros_spin(self):
        rospy.spin()

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
            feasibility = self.actions.generate_trj(int(self.static_gait_params['nstep']))
            if feasibility:
                # goto initial position
                #self.robot.move_all_legs(Vslow, self.Q[:,0])
                self.actions.demo()
                #print 'i should move to dragon position'
                return BhvSelectorResponse(True, 'Demo successfully set.')
            else:
                self.bhv_type = None
                return BhvSelectorResponse(False, 'Failed to set demo. Non feasible EndEff trajectory.')

        else:
            self.bhv_running = False
            self.bhv_type = None
            self.ctrl_timestep = 0.01
            return BhvSelectorResponse(False, 'Failed to set Behavior. Invalid bhv type selected.')



def handler(signum, frame):
    global should_quit
    should_quit = True

if __name__ == '__main__':
    rospy.init_node('uw_gripper')

    # prevhand is the handler set by the ROS runtime
    prevhand = signal.signal(signal.SIGINT, handler)
    controller = Runner()

    controller.loop()

    # calls ROS signal handler manually
    # this is to stop the rospy.spin thread
    prevhand(signal.SIGINT, None)        






