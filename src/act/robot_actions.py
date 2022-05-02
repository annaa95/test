#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Anna Astolfi

import numpy as np
from .robot_geom import RobotGeom


Q_idle = np.array([45,50,130,0,50,130,-45,50,130,-45,50,130,0,50,130,45,50,130]) #dragon position to idle
Q_idleH = np.array([45,0,90,0,0,90,-45,0,90,-45,0,90,0,0,90,45,0,90]) #dragon position to idle
#Q_idle = np.array([45,-75,0, 0,-75,0, -45,-75,0, -45,-75,0, 0,-75,0, 45,-75,0]) #low torque to idle
Q1 = np.array([45,30,120, 0,30,120, -45,30,120, -45,30,120, 0,30,120, 45,30,120]) #MC testing: pushing from dragon
Q2 = np.array([45,-20,70, 0,-20,70, -45,-20,70, -45,-20,70, 0,-20,70, 45,-20,70]) #MC testing: pushing from dragon
V0 = 30*np.ones([18], dtype='int')
Vslow = 1*np.ones([18], dtype='int')
dT = 2.5


class RobotAction():

    def __init__(self):

        # init robot
        N=6
        self.robot = RobotGeom()

        # demo parameters
        self.static_gait_params = None
        # create services
        #                available gaits
        self.set_gait_proxy = rospy.Service('silver_locomotion/set_gait', GaitSelector, self.set_gait)
        self.gait_start_proxy = rospy.Service('silver_locomotion/gait_start', Empty, self.gait_start)
        self.gait_stop_proxy = rospy.Service('silver_locomotion/gait_stop', Empty, self.gait_stop)
        #create publisher
        self.dxl_status_pub= rospy.Publisher('dxl_status', Float32MultiArray, queue_size=10)
        # subscribe to topics
        self.command_subscriber = rospy.Subscriber('silver_locomotion/cmd', Command, self.get_cmd)
        self.contact_subscriber = rospy.Subscriber('adc/contacts', Float32MultiArray, self.get_contacts)
        # static shared variables
        self.leg_ids = np.array([1,2,3,4,5,6])
        self.iteration_counter = 0
        self.T = np.zeros([18,120])
        self.Q = np.zeros([18,120])
        self.Qdot = np.zeros([18,120])
        # dynamic shared variable
        self.joint_vel_push = 0
        self.joint_vel_pull = 70*np.ones(6,dtype="int")
        
        self.q_femur_pull = np.zeros(6)
        self.q_femur_push = np.zeros(6)
        self.q_tibia_pull = np.zeros(6)
        self.q_tibia_push = np.zeros(6)
        self.q_coxa_pull = np.zeros(6)
        
        self.ts= 0 # tripod selector 0 : 0-2-4 , 1: 2-3-5
        self.vl_no = 1
        self.underactuated = 0
        self.rotation = 0
        self.feet_policy = "none"
        
        # ----crab variables (from crab experiments)-----

        self.numDataPoints = 100
        # remove dimensions from crab size by multipling by:
        self.lengthDimLess = 0.2908 # 1/[cm]
        self.timeDimLess = np.sqrt(1/(self.lengthDimLess*9.81)) #1/s
        # dimensionalize to Silver2 size by multipling by:
        self.lengthDimAdd = self.robot.l2+self.robot.l3 # check units !!
        self.timeDimAdd = np.sqrt(9.81/self.lengthDimAdd)
        # foot trajectories as ellipses
        self.amplitudeLeftX = 1.4*self.lengthDimLess*self.lengthDimAdd
        self.amplitudeLeftY = 0.55*self.lengthDimLess*self.lengthDimAdd
        self.amplitudeRigthX = 0.9*self.lengthDimLess*self.lengthDimAdd
        self.amplitudeRigthY = 0.55*self.lengthDimLess*self.lengthDimAdd
        self.centerLeft = np.array([1.9,-0.71])*self.lengthDimLess*self.lengthDimAdd
        self.centerRigth = np.array([2.0,-0.8])*self.lengthDimLess*self.lengthDimAdd
        self.frequency = 7.4227*(self.timeDimLess*self.timeDimAdd) #Hz

        #initial phase
        self.phi_Y=np.deg2rad(np.array([-229.1831, 0, -163.2930, -229.1831-65, -65, -163.2930-65]))        
        self.phi_X= self.phi_Y- np.deg2rad(np.array([36,36,36,-41.5,-41.5,-41.5]))

    def loop(self):
        rosthread = threading.Thread(target=self.ros_spin)
        rosthread.deamon = True
        rosthread.start()
        while True:

            if self.cc%200 == 0: #publish dxl status every 100 iterations
                dxl_status = Float32MultiArray()
                dxl_status.data = self.robot.get_status(np.arange(1,20))
                self.dxl_status_pub.publish(dxl_status)
            self.cc=self.cc+1

            if should_quit:
                break
            if self.gait_go:
                if self.gait_type == 'static':
                    #print 'faccio locomozione statica'
                    self.robot.move_all_legs(self.Qdot[:,self.iteration_counter], self.Q[:,self.iteration_counter])
                    self.iteration_counter = (self.iteration_counter+1)%int(self.static_gait_params['nstep'])

                elif self.gait_type == 'dynamic':
                    if self.touch_down():
                        self.time_last_contact = time.time()
                        self.robot.push(self.leg_ids[self.ts:6:self.dynamic_gait_params['vl_no'] ], self.joint_vel_push, self.q_femur_push, self.q_tibia_push, self.dynamic_gait_params['underactuated'])
                    elif self.lift_off():
                        self.time_last_detach = time.time()
                        if self.dynamic_gait_params['vl_no']==2:
                            #previously active tripods goest to idle
                            self.robot.pull(self.leg_ids[self.ts:6:self.dynamic_gait_params['vl_no'] ], self.joint_vel_pull, self.q_coxa_idle, self.q_femur_idle, self.q_tibia_idle, self.dynamic_gait_params['underactuated'])
                            #change active tripod
                            self.ts=(self.ts+1)%2
                            #currently active tripod performs pull
                            self.robot.pull(self.leg_ids[self.ts:6:self.dynamic_gait_params['vl_no'] ], self.joint_vel_pull, self.q_coxa_pull, self.q_femur_pull, self.q_tibia_pull, False) #non devo assare per torque enable
                        else:
                            self.robot.pull(self.leg_ids[self.ts:6:self.dynamic_gait_params['vl_no'] ], self.joint_vel_pull, self.q_coxa_pull, self.q_femur_pull, self.q_tibia_pull, self.dynamic_gait_params['underactuated'])
                
                elif self.gait_type =='crab':
                    self.robot.move_all_legs(self.Qdot[:,self.iteration_counter], self.Q[:,self.iteration_counter])
                    print(self.Q[:,self.iteration_counter])
                    self.iteration_counter = (self.iteration_counter+1)%int(self.numDataPoints) 
                else:
                    print('locomozione non selezionata')
            time.sleep(self.ctrl_timestep)

        if self.gait_go:
            self.gait_stop(None)

        self.robot.__del__()

    def ros_spin(self):
        rospy.spin()

    def set_crab(self, req):
        """
        set_crab is created as a set_gait with the only option of setting the crab like locomotion
        if experiments are promising, it is meant to be embedded in set_gait
        1- checks: maniulationMode? another gait running?
        2- read the request
        3- compute trajectories in 
        """
        if '/silver_locomotion/manipulation' in rosservice.get_service_list():
            return GaitSelectorResponse(False, 'Manipulation mode enabled')

        if self.gait_go:
            return GaitSelectorResponse(False, 'cannot set gait while moving')  

        if req.type == 'crab':
            self.gait_type = 'crab'
            self.crab_gait_params = {
                'frequency': req.frequency,
                'coxaAngle': req.coxaAngle,
                'ellipseRotation': req.ellipseRotation,
                'stretching': req.stretching,
                'axesToStretch': req.axesToStretch,
                'amplitudeScaling': req.amplitudeScaling
                }
            # check params
            if self.crab_gait_params['frequency']  <= 0 or self.crab_gait_params['frequency'] >10:
                self.crab_gait_params['frequency'] = self.frequency
            if self.crab_gait_params['coxaAngle']< 0 :
                self.crab_gait_params['coxaAngle'] = 5 # slightly opened coxa joints by default [deg]
            if self.crab_gait_params['ellipseRotation'] < -20 or self.crab_gait_params['ellipseRotation'] > 20:
                self.crab_gait_params['ellipseRotation'] = 0
            if self.crab_gait_params['stretching'] < 0.4 or self.crab_gait_params['stretching']> 2.5:
                self.crab_gait_params['stretching'] = 1
            if self.crab_gait_params['axesToStretch']== '' or self.crab_gait_params['axesToStretch'] != 'x' or (self.crab_gait_params['axesToStretch'] != 'X') or (self.crab_gait_params['axesToStretch'] != 'y') or (self.crab_gait_params['axesToStretch'] != 'Y'): 
                self.crab_gait_params['axesToStretch'] ='x'
            if self.crab_gait_params['amplitudeScaling'] < 0.25 or self.crab_gait_params['amplitudeScaling']> 2:
                self.crab_gait_params['amplitudeScaling'] = 1
            
            try:
                self.ctrl_timestep = 1/(self.crab_gait_params['frequency']*self.numDataPoints) # 1 centesimo della frequenza del movimento
            except ZeroDivisionError:
                self.ctrl_timestep = 0.01
            # compute trajectory, check feasibility and apply inverse kinematics
            self.T = np.zeros([2*6,self.numDataPoints]) #[x;y]*6legs 
            self.Q = np.zeros([3*6,self.numDataPoints]) 
            self.Qdot = np.zeros([3*6,self.numDataPoints])
            self.Qcoxa = np.deg2rad(np.array([self.crab_gait_params['coxaAngle'],0,-self.crab_gait_params['coxaAngle'],-self.crab_gait_params['coxaAngle'],0,self.crab_gait_params['coxaAngle']]))
            Admiss = [False]*6
            self.Ax = self.crab_gait_params['amplitudeScaling']*np.hstack((self.amplitudeRigthX*np.ones(3), self.amplitudeLeftX*np.ones(3)))
            self.Ay = self.crab_gait_params['amplitudeScaling']*np.hstack((self.amplitudeRigthY*np.ones(3), self.amplitudeLeftY*np.ones(3)))
            self.Cx = np.hstack((self.centerRigth[0]*np.ones(3), self.centerLeft[0]*np.ones(3)))
            self.Cy = np.hstack((self.centerRigth[1]*np.ones(3), self.centerLeft[1]*np.ones(3)))
            #rotate the ellipse according to ellipseRotation
            self.RotMat = np.array([[np.cos(np.deg2rad(self.crab_gait_params['ellipseRotation'])), np.sin(np.deg2rad(self.crab_gait_params['ellipseRotation']))],
                                        [-np.sin(np.deg2rad(self.crab_gait_params['ellipseRotation'])), np.cos(np.deg2rad(self.crab_gait_params['ellipseRotation']))]])
            for i in range(6):
                self.T[2*i:2*i+2,:],self.Q[3*i:3*i+3,:],self.Qdot[3*i:3*i+3,:],Admiss[i]= self.robot.crabFootPath(i, self.Qcoxa[i],self.numDataPoints,self.Ax[i],self.Ay[i],self.crab_gait_params['frequency'],self.Cx[i],self.Cy[i],self.phi_X[i], self.phi_Y[i], self.RotMat,self.crab_gait_params['axesToStretch'],self.crab_gait_params['stretching'])
            
            feasible = all(Admiss)
            if feasible:
                # goto initial position
                self.robot.move_all_legs(V0, self.Q[:,0])
                return GaitSelectionAnnaResponse(True, 'Crab gait successfully set.')
            else:
                self.gait_type = None
                return GaitSelectionAnnaResponse(False, 'Failed to set crab gait. Non feasible leg trajectory.')

    def set_gait(self, req):

        if '/silver_locomotion/manipulation' in rosservice.get_service_list():
            return GaitSelectorResponse(False, 'Manipulation mode enabled')

        if self.gait_go:
            return GaitSelectorResponse(False, 'cannot set gait while moving')

        if req.type == 'static':
            self.gait_type = 'static'
            self.static_gait_params = {
                'rotation': req.rotation,
                'gait_width': req.gait_width,
                'gait_height': req.gait_height,
                'direction': req.direction,
                'step_length': req.step_length,
                'duty_cycle': req.duty_cycle,
                'ground_clearance': req.ground_clearance,
                'phase_lag': np.array(req.phase_lag),
                'nstep': req.nstep,
                'period': req.period
                }
            # check params
            if self.static_gait_params['rotation']  < -1 or self.static_gait_params['rotation'] >1:
                self.static_gait_params['rotation'] = 0
            if self.static_gait_params['gait_width']<= 25 or self.static_gait_params['gait_width']> 60:
                self.static_gait_params['gait_width'] = 40
            if self.static_gait_params['gait_height'] <= 0:
                self.static_gait_params['gait_height'] = 40
            if self.static_gait_params['direction'] < -180 or self.static_gait_params['direction']>180:
                self.static_gait_params['direction'] = 90
            if self.static_gait_params['step_length'] <= 0:
                self.static_gait_params['step_length'] = 20
            if self.static_gait_params['duty_cycle'] <= 0 or self.static_gait_params['duty_cycle'] >= 1:
                self.static_gait_params['duty_cycle'] = 0.8
            if self.static_gait_params['ground_clearance'] <= 0:
                self.static_gait_params['ground_clearance'] = 10
            if self.static_gait_params['phase_lag'].size < 6:
                self.static_gait_params['phase_lag'] = np.array([90, 0, 90, 0, 90, 0])
            if self.static_gait_params['nstep'] <= 0:
                self.static_gait_params['nstep'] = 120
            if self.static_gait_params['period'] <= 0:
                self.static_gait_params['period'] = 10

            try:
                self.ctrl_timestep = self.static_gait_params['period'] / self.static_gait_params['nstep']
            except ZeroDivisionError:
                self.ctrl_timestep = 0.01
            # compute trajectory, check feasibility and apply inverse kinematics
            self.T = np.zeros([18,int(self.static_gait_params['nstep'])])
            self.Q = np.zeros([18,int(self.static_gait_params['nstep'])])
            self.Qdot = np.zeros([18,int(self.static_gait_params['nstep'])])
            Admiss = [False]*6
            for leg_id in range(0,6):
                self.T[3*leg_id:3*leg_id+3,:], self.Q[3*leg_id:3*leg_id+3,:], self.Qdot[3*leg_id:3*leg_id+3,:], Admiss[leg_id] = self.robot.trj_gen(self.static_gait_params['gait_width'], self.static_gait_params['gait_height'],self.static_gait_params['direction'],self.static_gait_params['step_length'],\
                                                                                                                          self.static_gait_params['duty_cycle'],self.static_gait_params['ground_clearance'],self.static_gait_params['phase_lag'][leg_id], int(self.static_gait_params['nstep']),self.static_gait_params['period'], leg_id, self.static_gait_params['rotation'])
            feasible = all(Admiss)
            if feasible:
                # goto initial position
                #self.robot.move_all_legs(Vslow, self.Q[:,0])
                self.robot.change_configuration(self.Q[:,0])
                #print 'i should move to dragon position'
                return GaitSelectorResponse(True, 'Static gait successfully set.')
            else:
                self.gait_type = None
                return GaitSelectorResponse(False, 'Failed to set static gait. Non feasible leg trajectory.')

        elif req.type == 'dynamic':
            self.gait_type = 'dynamic'
            #retrieve params from req
            self.dynamic_gait_params = {
            'joint_vel': req.joint_vel,
            'alpha': req.alpha,
            'td_height': req.td_height,
            'underactuated': req.underactuated,
            'vl_no': req.vl_no,
            'rotation': req.rotation,
            'feet_policy': req.feet_policy
            }
            #check and possibly assign to default
            if self.dynamic_gait_params['joint_vel']<= 0 or self.dynamic_gait_params['joint_vel']> 4000:
                self.dynamic_gait_params['joint_vel'] = 200
            if self.dynamic_gait_params['alpha']<=0 or self.dynamic_gait_params['alpha']>180 or self.dynamic_gait_params['alpha'] == 90:
                self.dynamic_gait_params['alpha'] = 89.9
            if self.dynamic_gait_params['td_height']<=0 or self.dynamic_gait_params['td_height']>60:
                self.dynamic_gait_params['td_height'] = 50
            if self.dynamic_gait_params['vl_no'] < 1 or self.dynamic_gait_params['vl_no'] >2:
                self.dynamic_gait_params['vl_no'] = 1
            if self.dynamic_gait_params['rotation']  < -1 or self.dynamic_gait_params['rotation'] >1:
                self.dynamic_gait_params['rotation'] =0

            self.dynamic_gait_params['alpha'] = self.dynamic_gait_params['alpha']*np.pi/180 #alpha from [deg] 2 [rad]
            self.joint_vel_push = self.dynamic_gait_params['joint_vel']*np.ones(6, dtype="int") # array of PUSH joint velocities [range 1-4000]
            #self.joint_vel_pull = 0.5*self.dynamic_gait_params['joint_vel']*np.ones(6, dtype="int") #  array of PULL joint velocities --> too 
            self.joint_vel_pull = 50*np.ones(6, dtype="int") #  array of PULL joint velocities #very slow


            #Handle combinations of underactuated | vl_no | rotation
            if self.dynamic_gait_params['rotation'] != 0: # ROTATIONS - ROTATIONS - ROTATIONS
                delta_s = 10 # feet horizontal displacement wrt coxa axis
                try:
                    [q2, q3] = self.robot.leg_inv_kine_coxa_plane([delta_s, self.dynamic_gait_params['td_height']])*180/np.pi
                except:
                    return GaitSelectorResponse('False', 'Failed to set dynamic rotation. Not admissible td_height')
                self.q_femur_pull = q2*np.ones(6)
                self.q_tibia_pull = q3*np.ones(6)
                self.q_femur_push = self.q_femur_pull
                #self.q_tibia_push = np.zeros(6)
                self.q_tibia_push = 20*np.ones(6)
                if self.dynamic_gait_params['rotation'] == -1: #clockwise
                    self.q_coxa_pull = np.array([90,45,0,0,45,90])
                else:#counter clockwise
                    self.q_coxa_pull = np.array([0,-45,-90,-90,-45,0])
            else: # FORWARD - FORWARD - FORWARD
                self.q_coxa_pull = np.array([0,0,0,0,0,0])  #coxe a 0
                if self.dynamic_gait_params['underactuated'] !=1: # ACTIVE FEMUR IN PUSHING
                    if self.dynamic_gait_params['feet_policy']=='equal':
                        dx = self.dynamic_gait_params['td_height']/np.tan(self.dynamic_gait_params['alpha'])# feet horizontal displacement wrt coxa axis
                        dl = 10 #leg linear elongation in direction alpha
                        L_ext = np.sqrt(dx**2+self.dynamic_gait_params['td_height']**2)+dl
                        if L_ext > self.robot.l2 + self.robot.l3:
                            L_ext = self.robot.l2 + self.robot.l3
                        try:
                            [q2_r, q3_r] = self.robot.leg_inv_kine_coxa_plane([-dx, self.dynamic_gait_params['td_height']])*180/np.pi #right side legs (1,2,3), retracted position
                            [q2_l, q3_l] = self.robot.leg_inv_kine_coxa_plane([dx, self.dynamic_gait_params['td_height']])*180/np.pi #left side legs (4,5,6), retracted position
                            [q2_r_ext, q3_r_ext] = self.robot.leg_inv_kine_coxa_plane([L_ext*np.cos(np.pi-self.dynamic_gait_params['alpha']), L_ext*np.sin(np.pi-self.dynamic_gait_params['alpha'])])*180/np.pi #right side legs (1,2,3), extended position
                            [q2_l_ext, q3_l_ext] = self.robot.leg_inv_kine_coxa_plane([L_ext*np.cos(self.dynamic_gait_params['alpha']), L_ext*np.sin(self.dynamic_gait_params['alpha'])])*180/np.pi #left side legs (1,2,3), extended position
                        except:
                            return GaitSelectorResponse(False, 'Failed to set dynamic gait. Not admissible td_height-alpha combination')
                        self.q_femur_pull = np.array([q2_r, q2_r, q2_r, q2_l, q2_l, q2_l])
                        self.q_tibia_pull = np.array([q3_r, q3_r, q3_r, q3_l, q3_l, q3_l])
                        self.q_femur_push = np.array([q2_r_ext, q2_r_ext, q2_r_ext, q2_l_ext, q2_l_ext, q2_l_ext])
                        self.q_tibia_push = np.array([q3_r_ext, q3_r_ext, q3_r_ext, q3_l_ext, q3_l_ext, q3_l_ext])
                    elif self.dynamic_gait_params['feet_policy']=='old': #OLD PUSHING, as in SciRo paper
                        delta_s = 20
                        leg_angles = np.array([1,1,1,-1,-1,-1])*45.0
                        q2 = 50
                        q3 =130
                        self.q_femur_pull = np.array([q2, q2, q2, q2, q2, q2])
                        self.q_tibia_pull = np.array([q3, q3, q3, q3, q3, q3])
                        p_pull = self.robot.kine_coxa_plane(np.array([q2, q3])*np.pi/180)
                        p_push = np.zeros([6,2])
                        for i in range(0,6):
                            p_push[i,:] = p_pull + np.array([delta_s*np.sin(leg_angles[i]),delta_s*np.cos(leg_angles[i])])
                            [self.q_femur_push[i], self.q_tibia_push[i]] = self.robot.leg_inv_kine_coxa_plane(p_push[i,:])*180/np.pi
                else: # UNDERACTUATED - UNDERACTUATED - UNDERACTUATED
                    if self.dynamic_gait_params['feet_policy']=='equal': #physical legs parallel to vl
                        dx_l = self.dynamic_gait_params['td_height']/np.tan(self.dynamic_gait_params['alpha'])# feet horizontal displacement wrt coxa axis
                        dx_r= -dx_l
                        #self.joint_vel_push = self.dynamic_gait_params['joint_vel']*np.ones(6, dtype="int")
                        # already specified in itialization of dynamics variable after the request
                    elif self.dynamic_gait_params['feet_policy']=='symmetric':
                        lin_disp = 5 #cm
                        dx = self.dynamic_gait_params['td_height']/np.tan(self.dynamic_gait_params['alpha'])
                        dx_l= dx+lin_disp
                        dx_r = -dx+lin_disp
                        alpha_r = np.pi - math.atan2(self.dynamic_gait_params['td_height'],dx_r) #[rad]
                        alpha_l = math.atan2(self.dynamic_gait_params['td_height'],dx_l) #[rad]
                        conv_factor =1 #digit --> deg/s
                        #interpolazione vuole alpha e omega in deg e deg/s
                        F_vl = 3.7610*(self.dynamic_gait_params['alpha']*180/np.pi)+ 1.1967*(self.dynamic_gait_params['joint_vel']*conv_factor) -246.3457
                        F = F_vl/(2*np.cos(alpha_r-self.dynamic_gait_params['alpha']))  #modulo della semiforza
                        omega_l = (2*F-3.7610*(90-np.abs(90-alpha_l*180/np.pi))+246.3457)/1.1967
                        omega_r = (2*F-3.7610*(90-np.abs(90-alpha_r*180/np.pi))+246.3457)/1.1967
                        self.joint_vel_push = np.array([omega_r,omega_r,omega_r,omega_l,omega_l,omega_l], dtype="int")
                        """
                        elif self.dynamic_gait_params['feet_policy']=='asymmetric': #one physical leg is kept to 90 deg
                        #probably this will be deleted, unused
                        alpha_l = 2*self.dynamic_gait_params['alpha']-np.pi/2
                        dx_l = self.dynamic_gait_params['td_height']/np.tan(alpha_l)
                        dx_r = 0
                        """
                    try:
                        # inv kine referred to femr joint x_foot = x_foot +self.l0, self.l0=8 
                        [q2_r, q3_r] = self.robot.leg_inv_kine_coxa_plane([dx_r+8, self.dynamic_gait_params['td_height']])*180/np.pi #right side legs (1,2,3), retracted position
                        [q2_l, q3_l] = self.robot.leg_inv_kine_coxa_plane([dx_l+8, self.dynamic_gait_params['td_height']])*180/np.pi #left side legs (4,5,6), retracted position
                    except:
                        return GaitSelectorResponse(False, 'Failed to set dynamic gait. Not admissible td_height-alpha combination.')
                    #-------------set Q_TD-set Q_TD-set Q_TD-------------
                    self.q_femur_pull = np.array([q2_r, q2_r, q2_r, q2_l, q2_l, q2_l])
                    self.q_tibia_pull = np.array([q3_r, q3_r, q3_r, q3_l, q3_l, q3_l])
                    #-------------set Q_LO-set Q_LO-set Q_LO-------------                    
                    self.q_femur_push = np.array([q2_r, q2_r, q2_r, q2_l, q2_l, q2_l])
                    #self.q_tibia_push= 0.7*self.q_tibia_pull #0.7 significa che estendo del 30%
                    Q3_LO = 30 #estensione va verso angolo q3=0
                    self.q_tibia_push = Q3_LO*np.ones(6, dtype="int") # tibia lift off can be tuned 

                    if self.dynamic_gait_params['vl_no'] == 2:#vl_no == 2, alternating tripod
                        try:
                            dz_idle = 20 #cm
                            dx_idle = 10 #cm
                            z = self.dynamic_gait_params['td_height']-dz_idle
                            [q2_i, q3_i] = self.robot.leg_inv_kine_coxa_plane([dx_idle, dz_idle])*180/np.pi #idle configuration
                        except:
                            return GaitSelectorResponse(False, 'IDLE ERROR Failed to set dynamic gait. Not admissible td_height-alpha combination.')
                        self.q_coxa_idle = np.zeros(6)
                        self.q_femur_idle = np.array([q2_i, q2_i, q2_i, q2_i, q2_i, q2_i])
                        self.q_tibia_idle = np.array([q3_i, q3_i, q3_i, q3_i, q3_i, q3_i])

            return GaitSelectorResponse(True, 'Dynamic gait successfully set.')
					
        elif req.type == 'push':
            #retrieve params from req
            self.push_params = {
            'push_vel': req.Dpush_vel,
            }
                        #check and possibly assign to default
            if self.push_params['push_vel']<= 0 or self.push_params['push_vel']> 4000:
                self.push_params['push_vel'] = 1
            print('Pulling action requested with speed:', self.push_params['push_vel'])
            self.robot.move_all_legs(self.push_params['push_vel']*np.ones([18], dtype='int'), Q1)
            #self.robot.goto_pose(self.push_params['push_vel']*np.ones([18], dtype='int'), Q2)
            print('Pulling action executed')
            # time.sleep(5)
            # self.robot.move_all_legs(Vslow, Q_idleH)
            #self.robot.goto_pose(V0, Q_idle)
            return GaitSelectorResponse(True, 'Pushing successfully completed')

        elif req.type == 'idleh':
            #just go in a higher sstance
            self.robot.move_all_legs(Vslow, Q_idleH)
            #self.robot.goto_pose(V0, Q_idle)
            return GaitSelectorResponse(True, 'High stance executed')

        else:
            self.gait_go = False
            self.gait_type = None
            self.ctrl_timestep = 0.01
            return GaitSelectorResponse(False, 'Failed to set gait. Invalid gait type selected.')

    def gait_start(self, req):
        if '/silver_locomotion/manipulation' in rosservice.get_service_list():
            return
            
        if self.gait_type is not None:
            if self.gait_type == "static":
                self.iteration_counter = 1
            elif self.gait_type == "dynamic":
                #inizializzare contatore per palleggio tripodi
                print("start dynamic gait")
                #primo salto
                self.robot.push(self.leg_ids[self.ts:6:self.dynamic_gait_params['vl_no'] ], self.joint_vel_push, self.q_femur_push, self.q_tibia_push, False)
            elif self.gait_type =='crab':
                self.iteration_counter = 1
                print('about to start crab gait')
            else:
                print("bad gait")
        else:
            print("non va bene")

        self.gait_go = True
        return EmptyResponse()

    def gait_stop(self, req):
        self.gait_go = False
        self.ctrl_timestep = 0.01
        print('gait stopped')
        # Go to idle position
        if self.gait_type == 'static':
            print('frozen yogurt')
	    #self.robot.change_configuration(Q_idle)
        else:
            self.robot.move_all_legs(V0, Q_idle)
        self.gait_type = None
        return EmptyResponse()

    def get_cmd(self, msg):
        self.last_cmd = time.time()
        self.dynamic_gait_cmd = {
            'velocity': msg.velocity,
            'extension': msg.extension,
            'angle_of_attack': msg.angle_of_attack,
            'heading': msg.heading,
            'yaw': msg.yaw
            }

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

    def sample(self, req):
        h = req.height
        l = req.length
        phi = -req.pitch*np.pi/180
        phi_recovery = 30*np.pi/180

        # --- [START] REFERENCE ---
        print('Go to reference position')
        # General parameters.
        ALL_s = np.arange(18)
        leg_num = self.robot.leg_num
        leg_joints = self.robot.leg_joints
            
        # Set initial displacement, orientation and feet position.
        #x_0, y_0, z_0, rad = 0, 0, 25, 15 Enrico
        x0, y0 = 0, 0
        rect_height = 10 #height of the rectangle
        z0 = h + rect_height #initial position is on the upper edge of the rectangle
        disp = [x0, y0, z0]
        orientation = [[1,0,0],[0,1,0],[0,0,1]]
        dist = 55#dist = 65
        #feet_angles_b = np.array([65, 0, 295, 115, 180, 245])*np.pi/180
        feet_angles_b = np.array([50, 0, 310, 130, 180, 230])*np.pi/180
        feet_pos_b = np.zeros(leg_num*leg_joints)
        feet_pos_l = np.zeros(leg_num*leg_joints)
        feet_q = np.zeros(leg_num*leg_joints)
        for i in range(leg_num):
            feet_pos_b[i*3:i*3+3] = dist*np.array([np.cos(feet_angles_b[i]), np.sin(feet_angles_b[i]), 0])
            feet_pos_l[i*3:i*3+3] = feet_pos_b[i*3:i*3+3]  - self.robot.leg_frame_b[i,:]  - disp
            if i > 2: #left side legs have frames rotated by 180deg
                feet_pos_l[i*3:i*3+2] = -feet_pos_l[i*3:i*3+2] # x and y change sign for the rotation
            try:   
                feet_q[i*3:i*3+3] = self.robot.leg_inv_kine(feet_pos_l[i*3:i*3+3],i)*180/np.pi
            except:
                return SamplingResponse(False, "inverse kinematic of leg failed - non admissible initial position")
                # return 
        self.robot.change_configuration(feet_q)
        
        # --- [START] SAMPLING TASK ---
        print('[Start] Sampling task')
        # Generate the path (rectangle) to follow.
        N_shortEdge, N_longEdge, period = 15, 45, 10.0 
        Npoints = 3*N_shortEdge+2*N_longEdge
        dt = period/Npoints
           
        path_x = x0*np.ones(Npoints)

        y1 = np.linspace(y0, y0-l/2,N_shortEdge)
        y2 = (y0-l/2)*np.ones(N_shortEdge)
        y3 = np.linspace(y0-l/2, y0+l/2, N_longEdge)
        y4 = (y0+l/2)*np.ones(N_longEdge)
        y5 = np.linspace(y0+l/2, y0, N_shortEdge)
        path_y = np.hstack((y1, y2, y3, y4, y5))

        z1 = z0*np.ones(N_shortEdge)
        z2 = np.linspace(z0, h, N_shortEdge)
        z3 = h*np.ones(N_longEdge)
        z4 = np.linspace(h,z0,N_longEdge)
        z5 = z0*np.ones(N_shortEdge)
        path_z = np.hstack((z1, z2, z3, z4, z5))
        
        path_psi = np.zeros(Npoints)
        
        path_theta = np.zeros(Npoints)

        phi1 = np.zeros(N_shortEdge)
        phi11 = np.linspace(0,phi,N_shortEdge/3)
        phi2 = phi*np.ones(2*N_shortEdge/3)
        phi3 = phi*np.ones(N_longEdge)
        phi4 = np.linspace(phi,phi_recovery, N_longEdge/3)
        phi41 = phi_recovery*np.ones(2*N_longEdge/3)
        phi5 = np.linspace(phi_recovery, 0, N_shortEdge)
        path_phi = np.hstack((phi1, phi11, phi2, phi3, phi4, phi41,phi5))
        
        Feet_local_history = []
        Q_path = np.zeros((path_x.size,leg_num*leg_joints))
        for i in range(path_x.size):
            # Set the orientation of the body.
            #psi, theta, phi = 0,0,0
            #orient_cur = [psi, theta, phi]
            orient_cur = [path_psi[i], path_theta[i], path_phi[i]]
            #phi = np.degrees(-np.arctan(path_z[np.mod(i+1,path_x.size)] - path_z[i]))
            # Set the displacement of the body from the ground.
            disp_cur = [path_x[i], path_y[i], path_z[i]]
            # Compute the leg (local) positions and joint angles [degrees].
            Feet_local = self.robot.ik_stewart(disp_cur, orient_cur,feet_pos_b)
            if i == 0:
                Feet_local_history = Feet_local
            else:
                Feet_local_history = np.vstack((Feet_local_history,Feet_local))
        # If the path is admissible within the legs joints space, follow the path.
        if self.robot.admissible(Feet_local_history):
            for i in range(path_x.size):
                # Get current pose of legs [degrees].
                Q_cur = np.degrees(self.robot.get_pose(ALL_s))
                Q_next = np.zeros(leg_num*leg_joints)
                for leg_id in range(leg_num):
                    try:
                        Q_next[leg_id*3:leg_id*3 + 3] = self.robot.leg_inv_kine(Feet_local_history[i*6 + leg_id,:],leg_id)
                    except:
                        return SamplingResponse(False, "inverse kinematic of leg failed")
                Q_next = np.degrees(Q_next)
                Qdot = np.abs(Q_next - Q_cur)/dt # la move all legs vuole sia Q che Qdot in gradi
                self.robot.move_all_legs(Qdot,Q_next)
                time.sleep(dt)
        else:
            return SamplingResponse(False, "non admissible path")
            # --- [END] SAMPLING TASK ---
        return SamplingResponse(True, "sampling task successfully completed")

    def set_manipulation(self,req):
        if self.gait_go:
            return SetManipulationResponse(False, 'Gait mode active')
        
        if '/silver_locomotion/manipulation' in rosservice.get_service_list():
            try:
                self.manipulation_proxy.shutdown()
            except:
                return SetManipulationResponse(False, 'Manipulation service not found')
            self.robot.change_configuration(Q_idle)
            return SetManipulationResponse(True, 'Manipulation mode disabled')

        else:
            self.manipulation_params = {
                'init_pos': np.array(req.init_position),	# initial displacement and orientation
                'feet_pos': np.array(req.feet_position)	    # feet global position
                }

            #check and possibly assign to default
            if not isinstance(self.manipulation_params['init_pos'],list) or len(self.manipulation_params['init_pos']) != 6:
                self.manipulation_params['init_pos'] = np.array([0,0,30,0,0,0])
            if self.manipulation_params['init_pos'][2] <= 0:
                self.manipulation_params['init_pos'][2] = 30
            if not isinstance(self.manipulation_params['feet_pos'],list) or len(self.manipulation_params['feet_pos']) != 18:
                self.manipulation_params['feet_pos'] = np.array([40,55,0,40,0,0,40,-55,0,-40,55,0,-40,0,0,-40,-55,0])

            self.current_state = self.manipulation_params['init_pos']
            Feet_local = self.robot.ik_stewart(self.current_state[0:3],self.current_state[3:6],self.manipulation_params['feet_pos'])
            Q = np.zeros(self.robot.leg_num*self.robot.leg_joints)
            for leg_id in range(self.robot.leg_num):
                try:
                    Q[leg_id*3:leg_id*3 + 3] = self.robot.leg_inv_kine(Feet_local[leg_id,:],leg_id)
                except:
                    return GaitSelectorResponse(False, "Inverse kinematic of leg failed")

            self.robot.change_configuration(np.degrees(Q))

            self.manipulation_proxy = rospy.Service('silver_locomotion/manipulation', Manipulation, self.manipulate)
            return SetManipulationResponse(True, 'Manipulation mode enabled')

    def manipulate(self,req):
        d_DoFs = np.array(req.d_DoFs)

        temp_DoFs = self.current_state + d_DoFs
        Feet_local = self.robot.ik_stewart(temp_DoFs[0:3], temp_DoFs[3:6],self.manipulation_params['feet_pos'])
                
        Q_cur = np.degrees(self.robot.get_pose(np.arange(0,18)))
        Q_next = np.zeros(self.robot.leg_num*self.robot.leg_joints)

        for leg_id in range(self.robot.leg_num):
            try:
                Q_next[leg_id*3:leg_id*3 + 3] = self.robot.leg_inv_kine(Feet_local[leg_id,:],leg_id)
            except:
                return ManipulationResponse(False, "Inverse kinematic of leg failed")

        Q_next = np.degrees(Q_next)
        Qdot = np.abs(Q_next - Q_cur) # la move all legs vuole sia Q che Qdot in gradi
        self.robot.move_all_legs(Qdot,Q_next)
        self.current_state = temp_DoFs

        return ManipulationResponse(True, "Correct positioning")

    def activate_hand(self, req):
        status = req.data
        if status == True:
            pos = 2250 #2500 for chicco's hand
        else:
            pos = 50 #500 for chicco's hand
        self.robot.manipulate([50], [pos]) #200 speed for chicco's hand
        return SetBoolResponse(True, "Successfully opened/closed the hand")

    def auto_sample(self, req):
        if self.gait_go:
            return SetManipulationResponse(False, 'Gait mode active')

        radius = 5 #cm
        dphi = np.pi/3 #angular increment
        phi = np.pi/6 #initial position
        dz = 0 #cm
        for count_samplings in range(6):
            # check dxl_status 
            for action in range(4):
                time.sleep(2) #2 seconds between execution of different loop actions
                if action == 0:
                    # go into cs_th position in xy-plane and upward on z or none movement
                    xyz = np.array([radius*np.cos(phi), radius*np.sin(phi), dz])
                elif action == 1:
                    # activate hand -open
                    self.robot.manipulate([50], [50])                    
                elif action == 2:
                    # go down 
                    dz = -10 #cm
                elif action == 3:
                    # deactivate hand -collect
                    self.robot.manipulate([50], [2250]) 
                    dz = +10 # to go up
                    phi += dphi # to translate

        return SetBoolResponse(True, "Successfully completed 6 point sampling")

if __name__ == '__main__':
    rospy.init_node('silver2')

    # prevhand is the handler set by the ROS runtime
    prevhand = signal.signal(signal.SIGINT, handler)
    controller = LocomotionController()

    controller.loop()

    # calls ROS signal handler manually
    # this is to stop the rospy.spin thread
    prevhand(signal.SIGINT, None)
