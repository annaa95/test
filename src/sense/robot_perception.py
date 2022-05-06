#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Anna Astolfi

from turtle import shape
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
        print("Successfully created a robot with %d legs" %self.robot.leg_num)
        # Goto pose just after creation
        self.goto_pose(self.robot.saved_velocities["medium"],self.robot.init_pose)       
    
    def read_encoders(self):
        encoder_values = []
        for i in range(self.robot.joint_num):
            encoder_values.append(self.robot.inOutFnct.get_pos(i))
        return encoder_values

    def goto_pose(self, vel, pose):
            """
            vel, pos are (leg.num xjoint_num)x1 vectors
            """
            for i in range(self.robot.leg_num):
                    for j in range(self.robot.joint_num):
                            print("pos: ", pose[j])
                            print("vel: ", vel[j])
                            self.robot.inOutFnct.set_pos(j,vel[j], pose[j])

    def generate_trj(self, n_pt = 120, Qi= None, Qf = None):
        """
        Qi = 3x1 vector of initial joint posiitons in radians
        Qf = 3x1 vector of final joint positions in radians
        n_pt = number of pts in the trjectory
        -----
        feasibility = admissibility check of the trajectory in the robot's workspace
        """
        if Qi == None:
            Qi = self.robot.q_min
        if Qf == None: 
            Qf = self.robot.q_max

        Q = np.zeros([3,n_pt])
        Qdot = np.ones([3,n_pt])   #1rad/s from step to step     
        for i in range(3):
            print("generating trj from ", Qi[i], "to ", Qf[i])
            Q[i,:] = np.linspace(Qi[i], Qf[i], n_pt)
        feasibility= self.robot.admissible_joint_space(Q)
        if not(feasibility):
            self.Q =None
            self.Qdot = None
        else:
            self.Q = Q
            self.Qdot = Qdot
            self.iter_num = 0
        return feasibility    
    
    def demo(self):
        print("trajecoty length:",len(self.Q))
        if self.iter_num< len(self.Q[0,:]):
            self.goto_pose(self.Qdot[:,self.iter_num], self.Q[:, self.iter_num])
            self.iter_num +=1
            print(self.iter_num)
            return True
        else:
            print("Demo completed !")
            return False
    
    def leg_joints_positioning(self, vel, q, leg):
            #Control the joint angles of specified leg to go to q (deg) with specified vel.
            joint_ids = self.get_leg_slice(self.motor_ids,leg)
            joint_vel = self.profvel_from_qdot(self.get_leg_slice(static_poses_vel[vel],leg))
            joint_pos = self.d_from_q(joint_ids,q)

            self.Q_cur[3*(leg-1):3*(leg-1)+3] = np.radians(q)
            self.robot_angles_and_status.publish(self.Q_cur,self.joints_status)

    def change_configuration(self, nq):
            nq = nq*np.pi/180
            oq = self.get_pose(np.arange(0,18))
            gait_t = 2 #[s]
            #assumption: configuration with neutral asset <--> all legs have the same height when they touch the ground
            new_leg_heights = np.zeros(6)
            old_leg_heights = np.zeros(6)
            for i in np.arange(0,6):
                np_i = self.kine(nq[3*i:3*i+3],i) #possibile fonte di errore
                op_i = self.kine(oq[3*i:3*i+3],i)
                new_leg_heights[i] = np_i[2]
                old_leg_heights[i] = op_i[2]
            nh = min(new_leg_heights)
            oh = min(old_leg_heights)
            if np.allclose(nq, oq):
                print("same pose as before")
                return True
            elif np.abs(nh-oh)<0.1:
                n1 = 0
                n2 = 15
            else:
                n1 = 30
                n2 = 15
            n = n1 + 2*n2
            t = np.linspace(0,1,n2)
            delta_t = gait_t/n
            Q = np.zeros([18,n])
            Qdot = np.zeros([18,n])
            admiss = np.ones(6, dtype=bool)
            
            for i in np.arange(0,6):
                nq_i = nq[3*i:3*i+3] #new configuration
                oq_i = oq[3*i:3*i+3] #old configuration
                np_i = self.kine(nq_i,i) #new foot tip position
                op_i = self.kine(oq_i,i) #old foot tip position
                #FIRST PHASE--> All legs go to new height with linear trajectory
                xi_1 = np.ones(n1)*op_i[0]
                yi_1 = np.ones(n1)*op_i[1]
                zi_1 = np.linspace(op_i[2], np_i[2], n1)
                #SECOND PHASE-->Tripod1 goes to new feet position with circumference arcs, Tripod2 stays
                if i==0 or i==2 or i==4:
                    xi_2 = np.linspace(op_i[0], np_i[0], n2)
                    yi_2 = np.linspace(op_i[1], np_i[1], n2)
                    zi_2 = np_i[2] + 5*np.sin(np.pi*t)
                else:
                    xi_2 = np.ones(n2)*op_i[0]
                    yi_2 = np.ones(n2)*op_i[1]
                    zi_2 = np.ones(n2)*np_i[2]
                #THIRD PHASE --> Tripod1 stays, Tripod2 goes to new feet position with circumference arcs
                if i==0 or i==2 or i==4:
                    xi_3 = np.ones(n2)*np_i[0]
                    yi_3 = np.ones(n2)*np_i[1]
                    zi_3 = np.ones(n2)*np_i[2]
                else:
                    xi_3 = np.linspace(op_i[0], np_i[0], n2)
                    yi_3 = np.linspace(op_i[1], np_i[1], n2)
                    zi_3 = np_i[2] + 5*np.sin(np.pi*t)
                # merge phases
                x_i = np.hstack((xi_1,xi_2,xi_3))
                y_i = np.hstack((yi_1,yi_2,yi_3))
                z_i = np.hstack((zi_1,zi_2,zi_3))
                T_i = np.vstack((x_i,y_i,z_i))
                # check admissibility and compute inverse kinematics and joint velocities
                admiss[i] = self.admissible(np.transpose(T_i))
                if admiss[i]:
                    for j in range(0,n):
                        Q[3*i:3*i+3,j] = self.leg_inv_kine(T_i[:,j], i) # leg_id=1...se metto i non funziona. C'è qualche incongruenza di segno sulle cinematiche
                    Qdot[3*i:3*i+3,0] = Q[3*i:3*i+3,n-1]-Q[3*i:3*i+3,0]
                    Qdot[3*i:3*i+3,1:n] = np.abs(np.diff(Q[3*i:3*i+3,0:n])/delta_t)
                else:
                    print('non admissible trj for leg_id: '+ str(i))
            # execute trajectory or terminate if trajectory is not admissible
            if all(admiss):
                for k in range(0, n):
                    self.move_all_legs(Qdot[:,k]*180/np.pi, Q[:,k]*180/np.pi)
                    time.sleep(delta_t)
                print("successfully changed pose")
                return True
            else:
                print("non admissible trajectories in transition")
                return False

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
