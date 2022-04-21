#!/usr/bin/env python
import sys
import numpy as np
# import robot_perceiving
# import robot_acting
# import robot_thinking

#define constants 

class Runner(object):
    def __init__(self):
        """
        Initialize the robot/simulated robot
        """
	self.actions = RobotAction() #qui saranno inizializzati i servizi/topic publisher...
	self.perception = RobotPerception() #qui saranno inizializzati i servizi/topic publisher...
	self.thoughts = RobotPlanning()#qui saranno inizializzati i servizi/topic publisher...

        
    def run(self):
        while True:
            print("I m running a boring controller.")
            #loop those three actions
	    #self.perception.perceive()
	    #self.thoughts.plan()
	    #self.action.act()
	
		
        
runner = Runner()
runner.run()





