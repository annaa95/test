#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Anna Astolfi
import numpy as np
import time
from interface_sense_sim import SimRobot
from interface_sense_rob import RealRobot

class RobotSensing():
    def __init__(self, 
                inputDevice = "Sim"):
        #manipulator input(?)
        
        # check if the output device is admissible
        if inputDevice == "Sim":
            self.inFnct = SimRobot("ee_pose_sensor")
        elif inputDevice == "Real":
            self.inFnct = RealRobot()
        else:
            print("None of admitted input devices. Picking the default one -> Webots ")
            self.inFnct = SimRobot()
        # init robot                     
        print(inputDevice)


    def get_contacts(self, msg):
        self.contacts[0:3] = msg.data[3:6]
        self.contacts[3:6] = msg.data[0:3]

    def get_dxl_status(self, req):
        motor_ids = np.arange(1,20)
        self.dxl_status = self.robot.get_status(motor_ids)
        return GetDxlStatusResponse(True, self.dxl_status, 'Status correctly retrieved')

    def reboot(self, req):
        if self.gait_go is True:
            return RebootResponse(False, [], 'Reboot not available when the robot is moving')
        else:
            self.get_dxl_status(None)
            dxl_id = np.where(self.dxl_status != 0)
            dxl_id = dxl_id[0] # the return of np.where is [[]]
            dxl_id = dxl_id + 1
            print("rebooting %r" % dxl_id)
            self.robot.reboot(dxl_id)
            time.sleep(5)
            print("enabling torque %r" % dxl_id)
            self.robot.torque_enable(dxl_id)
            self.get_dxl_status(None)
            return RebootResponse(True, dxl_id, 'Overload motors successfully reset')

    def touch_down(self):
        FeetFail=True
        workingfeet=np.array([4,5])
        if FeetFail: #to overcome not working feet but keeping the loop closed !!
            #td = self.contact is False and (time.time()-self.time_last_detach > self.t_min_pull) and (all(contact > self.td_threshold for contact in self.contacts[self.ts:6:self.dynamic_gait_params['vl_no'] ]) or time.time()-self.time_last_detach > self.t_max_pull)
            td = self.contact is False and (time.time()-self.time_last_detach > self.t_min_pull) and (any(contact > self.td_threshold for contact in self.contacts[workingfeet]) or time.time()-self.time_last_detach > self.t_max_pull)
        else:
            td = self.contact is False and (time.time()-self.time_last_detach > self.t_min_pull) and (any(contact > self.td_threshold for contact in self.contacts) or time.time()-self.time_last_detach > self.t_max_pull)
        if td:
            print("swimming phase lasted: " + str(time.time()-self.time_last_detach))
            self.contact = True
        return td

    def lift_off(self):
        FeetFail=True
        workingfeet=np.array([4,5])
        if FeetFail: #contact sensors do not work properly
            lo = self.contact is True and (time.time()-self.time_last_contact > self.t_min_push) and (any(contact < self.lo_threshold for contact in self.contacts[workingfeet]) or time.time()-self.time_last_contact > self.t_max_push)
            #lo = self.contact is True and (time.time()-self.time_last_contact > self.t_min_push) and (any(contact < self.lo_threshold for contact in self.contacts[self.ts:6:self.dynamic_gait_params['vl_no'] ]) or time.time()-self.time_last_contact > self.t_max_push)
        else: #we can rely on contact sensors
            lo = self.contact is True and (time.time()-self.time_last_contact > self.t_min_push) and (any(contact < self.lo_threshold for contact in self.contacts) or time.time()-self.time_last_contact > self.t_max_push)
        if lo:
            print("punting phase lasted: " + str(time.time()-self.time_last_contact))
            self.contact = False
        return lo

    def read_ee_pose(self):
        pose =np.zeros(6)
        for i in range(6):
            pose[i]=self.inFnct.get_ee_pose(i)
        return pose
