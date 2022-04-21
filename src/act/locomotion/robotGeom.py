#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Giacomo Picardi

from __future__ import division
import sys, os
import numpy as np
import numpy.matlib
from interfaceDevice import SimRobot, RealRobot
import dynamixel_communication.dynamixel as dxl
from locomotion.robot_def import *	# Constant definitions
import time
import math
from scipy.spatial import Delaunay
from scipy.linalg import svd
import transforms3d.euler as eul
import rospy
from silver_msgs.msg import PhantomVisualization
import pylab
import threading

class Robot:
        def __init__(self, 
                    leg_num = 6, 
                    leg_joints = 3, 
                    limits_file='/locomotion/trajectories/limits.txt', 
                    init_pose = 'folded', 
                    outputDevice = "Sim"): #manipulator input(?)
                
                # check if the output device is admissible
                if outputDevice == "Sim":
                    self.inOutFnct = SimRobot
                elif outputDevice == "Hard":
                    self.inOutFnct = RealRobot
                else:
                    print("None of admitted output devices. Picking the default")
                    self.inOutFnct = SimRobot

                #Structural characteristics of the robot.
                self.leg_num = leg_num			#Number of legs
                self.leg_joints = leg_joints	        #Number of joints in a leg
                self.joint_num = leg_num*leg_joints     #Number of joints in the robot
                
                self.l0 = 8 				#Lengths of leg segments in cm
                self.l1 = 7
                self.l2 = 30
                self.l3 = 32.3 	#26 without foot
                
                self.body_length = 40			#Dimensions of body in cm
                self.body_width = 55
                
                #[min,max] range of joints positioning in rad.
                self.q_min = np.array([-np.pi/2, -np.pi/2, -np.pi/4])
                self.q_max = np.array([np.pi/2, np.pi/2, 3/4*np.pi])

                # Modern Robotics approach requirements: 
                #   Mhome configurations (position of end effector when all joint are at 0 position) 4x4 matrix
                #   Sn = (ωn, vn) is the screw axis of joint n as expressed in the fixed base frame 6x1
                #Position of leg frames within the base frame.
                self.leg_frame_b = np.zeros([self.leg_num,3])
                self.leg_frame_b[0,:] = np.array([self.body_length/2, self.body_width/2, 0])
                self.leg_frame_b[1,:] = np.array([self.body_length/2, 0,0])
                self.leg_frame_b[2,:] = np.array([self.body_length/2, -self.body_width/2,0])
                self.leg_frame_b[3,:] = np.array([-self.body_length/2, self.body_width/2,0])
                self.leg_frame_b[4,:] = np.array([-self.body_length/2, 0,0])
                self.leg_frame_b[5,:] = np.array([-self.body_length/2, -self.body_width/2,0])
                #Leg frames and joints management.
                self.mir = np.array([1, 1, -1, -1, -1, 1])
                self.l_pos = np.array([1, 1, -1, 1, 1, -1])
                self.zeros = np.zeros(self.leg_joints*self.leg_num+1, dtype='int')
                self.sign = np.array([-1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, -1, -1, 1])
                
                #coxa joint rest position in deg.
                
                #self.rest_coxa_angle = np.zeros(6) #mrudul coxa 0, erase after experiments
                self.rest_coxa_angle = np.array([45, 0, -45, -45, 0, 45])*np.pi/180
                self.rest_coxa_angle_rot = np.array([60, 0, -60, -60, 0, 60])*np.pi/180
                #Create point cloud of leg workspace for admissibility test.
                nsample = 11
                self.cloud = np.zeros([nsample**3,3])
                c = 0
                q1 = np.linspace(self.q_min[0], self.q_max[0], nsample)
                q2 = np.linspace(self.q_min[1], self.q_max[1], nsample)
                q3 = np.linspace(self.q_min[2], self.q_max[2], nsample)
                for i in range(0,nsample):
                        for j in range(0,nsample):
                                for k in range(0,nsample):
                                        self.cloud[c,:] = self.kine([q1[i], q2[j], q3[k]],1) #patch rapida. Checko le gambe 3,4,5 sul workspace delle 1,2,6
                                        c = c+1
                self.Q_cur = np.zeros(self.leg_joints*self.leg_num)
                self.robot_angles_and_status = rospy.Publisher('robot_angles_and_status', PhantomVisualization, queue_size=10)
                time.sleep(1)
                #Generate a joint matrix: 0 corresponds to motor disable, 1 to motor enable.
                if manipulator == 0:
                        self.joints_status = np.zeros(self.leg_joints*self.leg_num)
                        self.motor_ids = np.arange(1,self.leg_joints*self.leg_num+1)
                else:
                        self.joints_status = np.zeros(self.leg_joints*self.leg_num+1)
                        self.motor_ids = np.arange(1,self.leg_joints*self.leg_num+2)
                if phantom==0:
                        #Setup motors.
                        self.motorHandler = dxl.Dynamixel(self.motor_ids)
                        self.motor_lock = threading.Lock() 
                #Store min and max joint positions from the file.
                minpos = np.zeros(self.leg_joints*self.leg_num+1, dtype='int')
                maxpos = np.zeros(self.leg_joints*self.leg_num+1, dtype='int')
                with open(limits_file,"r") as f:
                        counter = 0
                        for line in f:
                                fields = line.split(";")
                                self.zeros[counter] = fields[0]
                                minpos[counter] = fields[1]
                                maxpos[counter] = fields[2]
                                counter = counter + 1

                if phantom == 0:
                        self.motorHandler.set_pos_lim(self.motor_ids, minpos, maxpos)        
                # self.goto_pose("medium","low_torque")#modified by Giacomo for Anna's tests, previously "dragon"
                if phantom == 0:
                        print("Successfully created a robot with %d legs" %self.leg_num)
                else:
                        print("Successfully created a robot in phantom mode with %d legs" %self.leg_num)
                        print("Motors do not overload in phantom mode")
                # Goto pose just after creation
                self.goto_pose("medium",init_pose)

                
        def __del__(self):
                self.goto_pose("slow","low_torque") #to be used while outside water
                raw_input("Press ENTER to terminate")
                self.torque_disable(self.motor_ids)

        def torque_enable(self, motor_ids):
                self.joints_status[motor_ids-1] = 1
                if self.phantom == 0:
                        self.motor_lock.acquire()
                        self.motorHandler.torque_enable(motor_ids)
                        self.motor_lock.release()

        def torque_disable(self, motor_ids):
                self.joints_status[motor_ids-1] = 0
                if self.phantom == 0:
                        #self.motorHandler.torque_disable(ALL) #what about 'motor_ids' as input variable?
                        self.motorHandler.torque_disable(motor_ids)
                        
        def kine_coxa_plane(self, q):
                #q = [q1, q2]: joint coords of femur  and  tibia  joints in [rad]
                #p = [x_p, z_p]: foot tip position (2D) in the coxa plane  [cm]
                q2 = q[0]
                q3 = q[1]

                x_p = self.l0 + self.l2*np.cos(q2) + self.l3*np.cos(q2-q3)
                z_p = -self.l2*np.sin(q2) - self.l3*np.sin(q2-q3)

                return np.array([x_p, z_p])

        def kine(self, q, leg_id):  #updated 21/5/2020 to discriminate kinematics between legs 1,2,6 and 3,4,5
                q1 = q[0]
                q2 = q[1]
                q3 = q[2]

                x = (self.l0 + self.l2*np.cos(q2) + self.l3*np.cos(q2-q3))*np.cos(q1) - self.mir[leg_id]*self.l1*np.sin(q1)
                y = (self.l0 + self.l2*np.cos(q2) + self.l3*np.cos(q2-q3))*np.sin(q1) + self.mir[leg_id]*self.l1*np.cos(q1)
                z = self.l2*np.sin(q2) + self.l3*np.sin(q2-q3)

                return np.array([x, y, z])

        def leg_inv_kine_coxa_plane(self, p):
                #q = [q1, q2]: joint coords of tibia and femur joints
                #p = [x_p, z_p]: foot tip position (2D) in the coxa plane
                x_p = p[0]-self.l0 #tibia joint is shifted by l0 along x-axis in the coxa plane
                z_p = -p[1] #coxa plane has flipped z axis

                a = math.sqrt(x_p**2+z_p**2)
                q2 = math.acos((self.l2**2+a**2-self.l3**2)/(2*self.l2*a)) + math.atan2(z_p,x_p);
                q3 = math.pi - math.acos((self.l2**2+self.l3**2-a**2)/(2*self.l2*self.l3));
                return np.array([q2, q3]);


        def leg_inv_kine(self, tip_pos, leg_id):
                x = tip_pos[0]
                y = tip_pos[1]
                z = tip_pos[2]

                L = math.sqrt(x**2+y**2)
                beta = math.atan2(y,x);
                gamma = math.atan2(self.l1,math.sqrt(L**2-self.l1**2));
                q1 = beta - self.mir[leg_id]*gamma
                # x3,y3 are leg tip coords in coxa frame
                x3 = x*math.cos(q1)+y*math.sin(q1)-self.l0;
                y3 = z;
                a = math.sqrt(x3**2+y3**2);
                q2 = math.acos((self.l2**2+a**2-self.l3**2)/(2*self.l2*a)) + math.atan2(y3,x3);
                q3 = math.pi - math.acos((self.l2**2+self.l3**2-a**2)/(2*self.l2*self.l3));
                q = np.array([q1, q2, q3]);
                return q

        def admissible(self, T):
                """
                Test if points in `T` are in `hull`

                `T` should be a `NxK` coordinates of `N` points in `K` dimensions
                `hull` is either a scipy.spatial.Delaunay object or the `MxK` array of the
                coordinates of `M` points in `K`dimensions for which Delaunay triangulation
                will be computed
                """
                if not isinstance(self.cloud,Delaunay):
                        ws = Delaunay(self.cloud)
                        test = ws.find_simplex(T)>=0
                if np.size(test) == 1: #generalize to scalar case
                    return test
                else:
                    return all(test)
        
        def crabFootPath(self, leg_id, Qc,numPts, Ax,Ay,f,Cx,Cy,phi_X, phi_Y, Rotation,stretchXY, stretchAmount):
                t= np.linspace(0,1/f, numPts) # 1 periodo
                Q = np.zeros((3, numPts))
                Q[0,:] = Qc
                Qdot = np.zeros((3, numPts))
                X = Cx+Ax*np.cos(t+phi_X)
                Y = Cy+Ay*np.cos(t+phi_Y)                        
                XY = np.vstack((X,Y))
                if stretchAmount != 1:
                        XY = self.set_stretching(XY,stretchXY, stretchAmount) # (2,npoints)
                XY = np.transpose(np.dot(np.transpose(XY), Rotation))
                admiss = (np.amax(np.sqrt(np.sum(XY**2, 0))) <= self.l2+self.l3)#self.admissible(XY)
                if admiss:
                    for i in range(numPts):
                        qaux = self.leg_inv_kine_coxa_plane((XY[:,i]))
                        Q[1,i] = qaux[0]
                        Q[2,i] = qaux[1]
                    delta_t = 1/(f*numPts)
                    Qdot[:,0] = Q[:,numPts-1]-Q[:,0]
                    Qdot[:,1:numPts] = np.abs(np.diff(Q[:,0:numPts])/delta_t)
                else:
                    print('non admissible trj for leg_id: '+ str(leg_id))

                return XY, Q, Qdot, admiss

        def set_stretching(self, trj,stretchXY, stretchAmount):
                try:
                        U,_ , _ = svd(trj)
                except:
                        print('decomposition error')
                theta = -np.arctan2(U[1][0], U[0][0])
                TransfMat = np.array([[np.cos(theta), np.sin(theta)],[-np.sin(theta), np.cos(theta)]])
                XpYp = np.transpose(np.dot(np.transpose(trj),TransfMat))
                if stretchXY== 'x' or stretchXY== 'X':
                        print('stretching x')
                        stretchingMat = np.array([[stretchAmount, 0],[0,1]])
                elif stretchXY =='y' or stretchXY =='Y' :
                        stretchingMat = np.array([[1, 0],[0,stretchAmount]])
                else:
                        print('error: non existing axis')
                XsYs = np.transpose(np.dot(stretchingMat,XpYp))
                XfYf = np.transpose(np.dot(XsYs, np.transpose(TransfMat)))

                return XfYf

        def trj_gen(self,w,h,alpha,s,beta,delta_z,phi,n_step,gait_t,leg_id, rotation):
                """Gait parameters
                w = 40 #gait width [cm] ---> [x_c, y_c] for each leg
                h = 25 #gait height [cm]
                alpha = 90*np.pi/180 #direction [deg]
                s = 10 #step length [cm]
                beta = 2/3 #duty cycle
                delta_z = 10 #ground clearance [cm]
                phi = 0 #phase lag [deg]
                n_step = 120
                gait_t = 5 #period [s]
                leg_id = 0"""
                # check on parameter alpha
                if rotation == 0:
                    alpha = alpha*np.pi/180 # convert alpha to radians!!!!
                else:
                    if alpha == 0:
                        alpha = 1
                    alpha = alpha/np.abs(alpha) #when rotation is 1, alpha must be 1 or -1
                    s = s*np.pi/180

            	# find [x_c, y_c] for leg leg_id and gait_width w
                r = self.body_length/2 + w #radius of the circle centered in the middle of the body frame on which foot tips lie in rest postion
                d_leg = np.linalg.norm(self.leg_frame_b[leg_id,:]) #distance between leg_frame_b(leg_id) and body frame centre
                tip_coxa = np.array([(r - d_leg), self.mir[leg_id]*self.l1]) # (x,y) coords of foot tip in coxa frame
                R_coxa_leg = np.array([[np.cos(self.rest_coxa_angle[leg_id]), -np.sin(self.rest_coxa_angle[leg_id])],
        	    		       [np.sin(self.rest_coxa_angle[leg_id]), np.cos(self.rest_coxa_angle[leg_id])]])
                p_c = np.dot(R_coxa_leg,tip_coxa)
                x_c = p_c[0]
                y_c = p_c[1]
                n_s = int(n_step*beta)
                n_f = n_step-n_s
                
                if rotation == 0:
                    # stance phase trajectory
                    x_s = np.linspace(x_c + s/2*np.cos(alpha), x_c - s/2*np.cos(alpha), n_s)
                    y_s = np.linspace(y_c + s/2*np.sin(alpha), y_c - s/2*np.sin(alpha), n_s)
                    z_s = -h*np.ones([n_s])
                    # flying phase trajectory
                    t_f = np.linspace(0,1,n_f); #free parameter for trajectory definition
                    x_f = np.linspace(x_c - s/2*np.cos(alpha), x_c + s/2*np.cos(alpha), n_f)
                    y_f = np.linspace(y_c - s/2*np.sin(alpha), y_c + s/2*np.sin(alpha), n_f)
                    z_f = -(h - delta_z*np.sin(np.pi*t_f))
                else:
                    if leg_id < 3:
                        sign = -1
                    else:
                        sign = 1
                    # stance phase trajectory
                    t_s = alpha*np.linspace(-0.5,0.5,n_s)
                    x_s = r*np.cos((s*t_s+self.rest_coxa_angle_rot[leg_id]))+sign*self.leg_frame_b[leg_id,0]
                    y_s = r*np.sin((s*t_s+self.rest_coxa_angle_rot[leg_id]))+sign*self.leg_frame_b[leg_id,1]
                    z_s = -h*np.ones([n_s])
                    # flying phase trajectory
                    t_f = alpha*np.linspace(-0.5,0.5,n_f) #free parameter for trajectory definition
                    x_f = r*np.cos((-s*t_f+self.rest_coxa_angle_rot[leg_id]))+sign*self.leg_frame_b[leg_id,0]
                    y_f = r*np.sin((-s*t_f+self.rest_coxa_angle_rot[leg_id]))+sign*self.leg_frame_b[leg_id,1]
                    z_f = -(h - delta_z*np.cos(np.pi*t_f))
                    
                if rotation == 0 and leg_id > 2:
                    x_s = np.flipud(x_s)
                    y_s = np.flipud(y_s)
                    x_f = np.flipud(x_f)
                    y_f = np.flipud(y_f)
		
                # merge phases
                x = np.hstack((x_s,x_f))
                y = np.hstack((y_s,y_f))
                z = np.hstack((z_s,z_f))
                T = np.vstack((x,y,z))
                # check admissibility and compute inverse kinematics and joint velocities
                Q = np.zeros([3, n_step])
                Qdot = np.zeros([3, n_step])
                admiss = self.admissible(np.transpose(T))
                if admiss:
                    for i in range(0,n_step):
                        Q[:,i] = self.leg_inv_kine(T[:,i], leg_id)
                    delta_t = gait_t/n_step
                    Qdot[:,0] = Q[:,n_step-1]-Q[:,0]
                    Qdot[:,1:n_step] = np.abs(np.diff(Q[:,0:n_step])/delta_t)
                else:
                    print('non admissible trj for leg_id: '+ str(leg_id))
        		# apply phase lag
                index = int(n_step*(phi/360))
                Q_tmp = np.zeros(Q.shape)
                Qdot_tmp = np.zeros(Qdot.shape)
                T_tmp = np.zeros(T.shape)
                Q_tmp[:, 0:index] = Q[:,n_step-index:n_step]
                Q_tmp[:,index:n_step] = Q[:,0:n_step-index]
                Qdot_tmp[:, 0:index] = Qdot[:,n_step-index:n_step]
                Qdot_tmp[:,index:n_step] = Qdot[:,0:n_step-index]
                T_tmp[:, 0:index] = T[:,n_step-index:n_step]
                T_tmp[:,index:n_step] = T[:,0:n_step-index]
                
                return T_tmp, Q_tmp*180/np.pi, Qdot_tmp*180/np.pi, admiss

        def get_leg_slice(self, data, leg_id):
                #Data: array of size leg_joints*joint_num
                #leg_id: id of a leg
                #return: a slice of data corresponding to the joints in leg with id leg_id
                #leg_id=1--->[data[1], data[2], data[3]]
                return data[self.leg_joints*(leg_id-1):self.leg_joints*(leg_id-1)+self.leg_joints]

        def d_from_q(self, motor_ids, q):
                #q: joint coordinate in [deg]
                #d: converts q to goal position command of dynamixel
                #it requires the zero calibrated (or passed) in the contructor and motor id
                try: #handles case of single motor
                        d = np.zeros(len(motor_ids), dtype = 'int')
                except:
                        d = self.sign[motor_ids-1]*q/POS_UNIT+self.zeros[motor_ids-1]
                        return d
                current = 0
                for i in motor_ids:
                        d[current] = self.sign[i-1]*q[current]/POS_UNIT+self.zeros[i-1]
                        current = current+1
                return d

        def q_from_d(self, motor_ids, d):
                #inverse of d_from_q
                q = np.zeros(len(motor_ids))
                current = 0
                for i in motor_ids:
                        q[current] = self.sign[i-1]*(d[current] - self.zeros[i-1])*POS_UNIT
                        current = current+1
                return q

        def profvel_from_qdot(self, qdot):
                #converts joint velocity qdot in deg/s to profvel motor command
                profvel =  qdot*6/VEL_UNIT
                return profvel.astype(int)

        def qdot_from_profvel(self, qdot):
                #inverse of profvel_from_qdot
                return qdot*VEL_UNIT/6

        def foot_tip_positioning(self, vel, tip_pos, leg):
                #Control the foot tip of specified leg to go to tip_pos with specified vel.
                joint_ids = self.get_leg_slice(self.motor_ids,leg)
                joint_vel = self.profvel_from_qdot(self.get_leg_slice(static_poses_vel[vel],leg))
                q = self.leg_inv_kine(tip_pos, leg)*180/math.pi
                joint_pos = self.d_from_q(joint_ids,q)
                
                self.Q_cur[3*(leg-1):3*(leg-1)+3] = np.radians(q)
                self.robot_angles_and_status.publish(self.Q_cur,self.joints_status)
                if self.phantom == 0:
                        self.motor_lock.acquire()
                        self.motorHandler.allocate_conf(joint_ids, joint_vel, joint_pos)
                        self.motorHandler.set_conf()
                        self.motor_lock.release()

        def leg_joints_positioning(self, vel, q, leg):
                #Control the joint angles of specified leg to go to q (deg) with specified vel.
                joint_ids = self.get_leg_slice(self.motor_ids,leg)
                joint_vel = self.profvel_from_qdot(self.get_leg_slice(static_poses_vel[vel],leg))
                joint_pos = self.d_from_q(joint_ids,q)

                self.Q_cur[3*(leg-1):3*(leg-1)+3] = np.radians(q)
                self.robot_angles_and_status.publish(self.Q_cur,self.joints_status)
                if self.phantom == 0:
                        self.motor_lock.acquire()
                        self.motorHandler.allocate_conf(joint_ids, joint_vel, joint_pos)
                        self.motorHandler.set_conf()
                        self.motor_lock.release()

        def goto_pose(self, vel, pose):
                for i in range(1,self.leg_num+1):
                        joint_ids = self.get_leg_slice(self.motor_ids,i)
                        joint_vel = self.profvel_from_qdot(self.get_leg_slice(static_poses_vel[vel],i))
                        joint_pos = self.d_from_q(joint_ids,self.get_leg_slice(static_poses_pos[pose],i))
                        
                        self.Q_cur[3*(i-1):3*(i-1)+3] = np.radians(self.get_leg_slice(static_poses_pos[pose],i))
                        self.robot_angles_and_status.publish(list(self.Q_cur),list(self.joints_status))

                        if self.phantom == 0:
                                self.motor_lock.acquire()
                                self.motorHandler.allocate_conf(joint_ids, joint_vel, joint_pos)
                                self.motorHandler.set_conf()
                                self.motor_lock.release()
                        
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

        def get_pose(self, motor_ids):
                return self.Q_cur[motor_ids]
                """if self.phantom:
                        print self.Q_cur[motor_ids]
                        return self.Q_cur[motor_ids]
                else:
                        print np.radians(self.q_from_d(motor_ids,self.motorHandler.get_pos(motor_ids)))
                        return np.radians(self.q_from_d(motor_ids,self.motorHandler.get_pos(motor_ids)))"""

        def get_status(self, motor_ids):
                if self.phantom:
                        return np.zeros(len(motor_ids))
                else:
                        self.motor_lock.acquire()
                        status = self.motorHandler.get_status(motor_ids)
                        self.motor_lock.release()
                        return status

        def reboot(self, motor_ids):
                if self.phantom:
                        print("No reboot in phantom mode")
                        return np.zeros(len(motor_ids))
                else:
                        self.motor_lock.acquire()
                        self.motorHandler.reboot(motor_ids)
                        self.motor_lock.release()

        def move_all_legs(self, vel, pos):
                #Marce's version (Sept 2019)
                joint_vel = self.profvel_from_qdot(vel)
                joint_pos = self.d_from_q(self.motor_ids[0:18],pos)
                for c in range(0,6):

                        self.Q_cur[3*c:3*c+3] = np.radians(pos[c*3:c*3+3:1])
                        self.joints_status = np.ones(18, dtype=int)
                        self.robot_angles_and_status.publish(list(self.Q_cur),list(self.joints_status))
                        
                        if self.phantom == 0: 
                                self.motor_lock.acquire()
                                self.motorHandler.allocate_conf(self.motor_ids[c*3:c*3+3:1], joint_vel[c*3:c*3+3:1], joint_pos[c*3:c*3+3:1])
                                self.motorHandler.set_conf()
                                self.motor_lock.release()
                                        
        def push(self, leg_ids, joint_vel, q_femur, q_tibia, underactuated):
                #q_femur, q_tibia [deg] - final position, array of length = length(leg_ids)
                #joint_vel [digit] - angular velocity of joints, array of length = length(leg_ids)
                #underactuated [binary] - if 1 femur off
                coxa_joint_ids = leg_ids*3-2
                femur_joint_ids = leg_ids*3-1
                tibia_joint_ids = leg_ids*3
                print(self.phantom)
                for i in np.arange(0,len(leg_ids)): 
                        if self.phantom == 0: # sending motor commands to robot
                                self.motor_lock.acquire()
                                if underactuated:
                                        #print('switching off motors', femur_joint_ids[i])
                                        self.torque_disable(femur_joint_ids[i]) #the disable function manages the phantom case automatically
                                self.motorHandler.allocate_conf(femur_joint_ids[i], joint_vel[leg_ids[i]-1], self.d_from_q(femur_joint_ids[i], q_femur[leg_ids[i]-1]))
                                self.motorHandler.allocate_conf(tibia_joint_ids[i], joint_vel[leg_ids[i]-1], self.d_from_q(tibia_joint_ids[i], q_tibia[leg_ids[i]-1]))
                                self.motorHandler.set_conf()
                                self.motor_lock.release()

                        # PHANTOM : set active joints status to 1
                        self.joints_status[coxa_joint_ids[i]-1]=1
                        self.joints_status[tibia_joint_ids[i]-1]=1
                        # PHANTOM : set motor angles 
                        self.Q_cur[femur_joint_ids[i]-1] = np.radians(q_femur[leg_ids[i]-1]) #ignored if status ==0
                        self.Q_cur[tibia_joint_ids[i]-1] = np.radians(q_tibia[leg_ids[i]-1])
                        self.robot_angles_and_status.publish(self.Q_cur,self.joints_status)
                      

        def pull(self, leg_ids, joint_vel, q_coxa, q_femur, q_tibia, underactuated):
                #q_femur, q_tibia, q_coxa [deg] - final position, array of length = length(leg_ids)
                #joint_vel [digit] - angular velocity of joints, array of length = length(leg_ids)
                # during pull there is the previously active tripod becoming idle and the other one preparing for landing
                coxa_joint_ids = leg_ids*3-2
                femur_joint_ids = leg_ids*3-1
                tibia_joint_ids = leg_ids*3

                for i in np.arange(0,len(leg_ids)): #: np.array([3,6])-1 per prove banco
                        if underactuated: # se fully actuted non succede nulla perchè è già tutto acceso
                                self.torque_enable(femur_joint_ids[i])
                                #print('switching on motors', femur_joint_ids[i])
                        if self.phantom == 0:
                                self.motor_lock.acquire()
                                self.motorHandler.allocate_conf(femur_joint_ids[i], joint_vel[leg_ids[i]-1], self.d_from_q(femur_joint_ids[i], q_femur[leg_ids[i]-1]))
                                self.motorHandler.allocate_conf(tibia_joint_ids[i], joint_vel[leg_ids[i]-1], self.d_from_q(tibia_joint_ids[i], q_tibia[leg_ids[i]-1]))
                                self.motorHandler.allocate_conf(coxa_joint_ids[i], joint_vel[leg_ids[i]-1], self.d_from_q(coxa_joint_ids[i], q_coxa[leg_ids[i]-1]))
                                self.motorHandler.set_conf()
                                self.motor_lock.release()
                        # PHANTOM: set motor angles (DONE ANYWAY)
                        self.Q_cur[femur_joint_ids[i]-1] = np.radians(q_femur[leg_ids[i]-1])
                        self.Q_cur[tibia_joint_ids[i]-1] = np.radians(q_tibia[leg_ids[i]-1])
                        self.Q_cur[coxa_joint_ids[i]-1] = np.radians(q_coxa[leg_ids[i]-1])
                        self.robot_angles_and_status.publish(self.Q_cur,self.joints_status)
                        
        def set_coxa(self, leg_ids, joint_vel, joint_pos):
                #..._joint_vel [digit]
                #..._joint_pos [deg]
                #SET COXA ACTION: set coxa joint at joint_pos position with joint_vel velocity
                joint_ids = leg_ids*3-2
                self.Q_cur[joint_ids-1] = np.radians(joint_pos)
                self.robot_angles_and_status.publish(self.Q_cur,self.joints_status)
                if self.phantom == 0:
                        self.motor_lock.acquire()
                        self.motorHandler.allocate_conf(joint_ids, joint_vel, self.d_from_q(joint_ids, joint_pos))
                        #set positions previously allocated
                        self.motorHandler.set_conf()
                        self.motor_lock.release()

        def ik_stewart(self, displacement, orientation, F_b):
                rotation = eul.euler2mat(orientation[0],orientation[1],orientation[2],axes='szyx')
                #Anchor points of legs in the base frame
                A_b = np.tile(displacement,self.leg_num) + np.reshape(rotation.dot(self.leg_frame_b.T).T,18) 
                #Feet tip position in leg frames
                F_l = rotation.T.dot(np.reshape(F_b - A_b,(6,3)).T) 			
                #Mirror left side legs.
                F_l[0,self.leg_num//2:self.leg_num + 1] = -F_l[0,self.leg_num//2:self.leg_num + 1]
                F_l[1,self.leg_num//2:self.leg_num + 1] = -F_l[1,self.leg_num//2:self.leg_num + 1]
                return F_l.T

        def manipulate(self, vel, pos):
                self.motor_lock.acquire()
                self.motorHandler.allocate_conf([19], vel, pos)
                #set positions previously allocated
                self.motorHandler.set_conf()
                self.motor_lock.release()
