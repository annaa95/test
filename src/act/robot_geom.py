#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Author: Anna Astolfi

from __future__ import division
from itertools import product
from scipy.spatial import Delaunay
from scipy.linalg import svd
import math
import matplotlib.pyplot as plt
import numpy as np
import os, sys
import time
import transforms3d.euler as eul

#from .robot_def import *	# Constant definitions


fileDir = os.path.dirname(os.path.abspath(__file__))
parentDir = os.path.dirname(fileDir)
path = sys.path
path.append(parentDir)

class RobotGeom():
        def __init__(self, 
                    leg_num = 1, # 1 because only manipulator
                    leg_joints = 3, 
                    joint_limits = "/home/anna/catkin_ws/src/silver3/src/act/trajectories/limits.txt",
                    init_pose = 'folded'): #manipulator input(?)
                
                #Structural characteristics of the robot.
                self.leg_num = leg_num			#Number of legs
                self.leg_joints = leg_joints	        #Number of joints in a leg
                self.joint_num = leg_num*leg_joints     #Number of joints in the robot
                
                self.l0 = 2.5 				#Lengths of leg segments in cm
                self.l1 = 7
                self.l2 = 32
                self.l3 = 25 	
                
                self.body_length = 40			#Dimensions of body in cm
                self.body_width = 55
                
                #[min,max] range of joints positioning in rad.
                self.q_min = np.zeros(self.joint_num)
                self.q_max = np.zeros(self.joint_num)
                #read from txt file
                with open(joint_limits,"r") as f:
                        counter = 0
                        for line in f:
                                fields = line.split(";")
                                #self.zeros[counter] = fields[0]
                                self.q_min[counter] = fields[1]
                                self.q_max[counter] = fields[2]
                                counter = counter + 1
                # Modern Robotics approach requirements: 
                #   Mhome configurations (position of end effector when all joint are at 0 position) 4x4 matrix

                self.home_matrices= np.zeros(shape=(self.leg_num,16))
                for i in range(self.leg_num):
                        R_i = np.reshape(np.eye(4), (1,16))
                        self.home_matrices[i,:] = R_i
                        self.home_matrices[i, -2] = self.l0+self.l1+self.l2+self.l3
                
                #   Sn = (??n, vn) is the screw axis of joint n as expressed in the fixed base frame 6x1
                omega = np.array([[0,1,0], [0, 0, 1], [0,0,1]])
                pos_joints = np.array([[0, self.l0, 0], [0, self.l0+self.l1, 0], [0,self.l0+self.l1+self.l2,0]])
                self.screw_axis = np.zeros([6,self.joint_num])    
                for i in range(self.joint_num):
                        self.screw_axis[0:3,i] = np.transpose(omega[i])
                        self.screw_axis[3:6,i] = np.cross(-np.transpose(omega[i]), np.transpose(pos_joints[i]))

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
                                        self.cloud[c,:] = self.forward_kinematics([q1[i], q2[j], q3[k]],leg_id=0) #patch rapida. Checko le gambe 3,4,5 sul workspace delle 1,2,6
                                        c = c+1                
                self.saved_poses = {
                        "extended": np.zeros(self.joint_num),
                        "folded": self.q_min,
                        "low_torque": self.q_min,
                        }
                self.saved_velocities = {
                        "slow": np.ones(self.joint_num),
                        "medium": 5*np.ones(self.joint_num),
                        "fast": 10*np.ones(self.joint_num),
                } #rad/s

                self.init_pose = self.saved_poses[init_pose]
                print("init pose: ", self.init_pose)
                time.sleep(1)
                
        def adjoint_representation(self, joint_id, q):
                """
                This function returns the i-th column of the jacobian 
                joint_id
                q = joint positions
                """
                if joint_id == 1:
                        J_si = self.screw_axis[:,joint_id]
                else:
                        T = np.eye(4) #see pag 100 Modoern Robotics
                        for i in range(joint_id-1, 0, -1):
                                # A*B*C = (A*B)*C = A*(B*C) propriet?? associativa prodotto tra matrici
                                T *= self.product_of_exponentials(screw_axis=self.screw_axis[:,i], displacement= q[i]) #homogeneous transformation from joint 0 to joint i-1
                        R = product[0:3,0:3]
                        p = product[0:3, 3]
                        p_skew = self.v_to_vskew(p)
                        Ad_T = np.zeros((6,6))
                        Ad_T[0:3,0:3] = R 
                        Ad_T[3:6,3:6] = R
                        Ad_T[3:6,0:3] = p_skew*R 
                        J_si = Ad_T*self.screw_axis[:,i]
                return J_si

        def get_jacobian(self, q):
                self.J =np.zeros((6, self.joint_num))    
                for i in range(self.joint_num):
                        self.J[:,i] = self.adjoint_representation(i, q)
                
        def v_to_vskew(self,v):
                v_skew = np.reshape(np.array([0, -v[2], v[1],   \
                                        v[2], 0, -v[0],         \
                                        -v[1],v[0], 0]),[3,3])
                return v_skew
           
        def product_of_exponentials(self, screw_axis, displacement):
                """
                Exponential coordinates of a homegeneous tranformation
                [screw_axis]*displacement in se(3) -> T in SE(3)
                Displacement: distance to be traveled along the screw axis to take the frame
                from the origin I to T
                if norm(omega) ==1, the displacement is the angle of rotation about the screw axis
                is omega==0 and norm(v) ==1 then the displacement is a linear distance along the
                screw axis
                """
                omega = screw_axis[0:3]
                omega_skew = self.v_to_vskew(omega)
                v = screw_axis[3:6]
                th = displacement
                G= np.eye(3)*th+ (1-np.cos(th))*omega_skew+(th-np.sin(th))*omega_skew**2
                Gv = np.reshape(G.dot(v), (3,1))
                C =self.exponential_coordinate_representation(omega_skew, th)
                HT = np.hstack((C, Gv))
                homogeneous_transformation = np.vstack((HT, np.array([0,0,0,1])))
                
                return homogeneous_transformation
        
        def exponential_coordinate_representation(self, skew_matrix, displacement):
                C = np.eye(3)+np.sin(displacement)*skew_matrix+(1-np.cos(displacement))*skew_matrix**2
                return C
        
        def forward_kinematics(self, q, leg_id=0, to_joint=2, from_joint=0):
                home_matrix = np.transpose(np.reshape(self.home_matrices[leg_id], [4,4]))
                product = np.eye(4)
                for i in range(to_joint, from_joint, -1):
                        # A*B*C = (A*B)*C = A*(B*C) propriet?? associativa prodotto tra matrici
                        product *= self.product_of_exponentials(screw_axis=self.screw_axis[:,i], displacement= q[i])
                T = product * home_matrix
                x = T[0,3]
                y = T[1,3]
                z = T[2,3]
                return np.array([x, y, z])
        
        def inverse_kinematics(self, P, leg_id=0):
                x = P[0]
                y = P[1]
                z = P[2]
                q0 = math.atan2(z,x)

                r = np.sqrt(x**2+z**2)
                L = math.sqrt(r**2+(y-self.l1)**2)
                print((self.l2**2+self.l3**2-L**2)/(2*self.l2*self.l3))
                q2 = math.pi - math.acos((self.l2**2+self.l3**2-L**2)/(2*self.l2*self.l3))

                beta = math.asin(math.sin(math.pi-q2)*self.l3/L)
                q1 = math.asin(r/L) - beta 
                
                q = np.array([q0, q1, q2])
                return q                
        
        def leg_inv_kine_coxa_plane(self, p):
                #q = [q1, q2]: joint coords of tibia and femur joints
                #p = [x_p, z_p]: foot tip position (2D) in the coxa plane
                x_p = p[0]-self.l0 #tibia joint is shifted by l0 along x-axis in the coxa plane
                z_p = -p[1] #coxa plane has flipped z axis

                a = math.sqrt(x_p**2+z_p**2)
                q2 = math.acos((self.l2**2+a**2-self.l3**2)/(2*self.l2*a)) + math.atan2(z_p,x_p);
                q3 = math.pi - math.acos((self.l2**2+self.l3**2-a**2)/(2*self.l2*self.l3));
                return np.array([q2, q3])



        def admissible_cart_space(self, T):
                try:
                        isinstance(self.cloud,Delaunay)
                        if not isinstance(self.cloud,Delaunay):
                                try:
                                        ws = Delaunay(self.cloud)
                                        try:
                                                test = ws.find_simplex(T)>=0
                                                if np.size(test) == 1: #generalize to scalar case
                                                        return test
                                                else:
                                                        return all(test)
                                        except:
                                                print("Error in find simplex")

                                except:
                                        print("Error in delaunay")
                except:
                        print("isinstance error")
                        return False


        
        def admissible_joint_space(self, Q):
                for i in range(3):
                        for j in range(len(Q)):
                                if Q[i,j]< self.q_min[i]:
                                        return False
                                elif Q[i,j] > self.q_max[i]:
                                        return False
                return True
                                        
                
        def cartesian2joint_traj(self, T, dT):
                """
                Traduzione di una traiettoria T = [x,y,z](t) Nx3 (with N time frames, and 3 coordinate end effectors)
                with dT = time to complete the trjectory in seconds
                in angoli di giunto Q[rad], velocit?? Qdot [rad/s] e check ammissibilit??
                """
                # check admissibility and compute inverse kinematics and joint velocities
                Q = np.zeros([3, len(T)])
                Qdot = np.zeros([3, len(T)])
                admiss = self.admissible(np.transpose(T))
                if admiss:
                    for i in range(len(T)):
                        Q[:,i] = self.inv_kine(T[:,i])
                    delta_t = dT/len(T)
                    Qdot[:,0] = Q[:,len(T)-1]-Q[:,0]
                    Qdot[:,1:len(T)] = np.abs(np.diff(Q[:,0:len(T)])/delta_t)
                else:
                    print('non admissible trj for leg_id: ')
                
                return Q, Qdot, admiss
        
        def static_OL_traj(self,w,h,alpha,s,beta,delta_z,phi,n_step,gait_t,leg_id, rotation):
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
                return T

        def get_leg_slice(self, data, leg_id):
                #Data: array of size leg_joints*joint_num
                #leg_id: id of a leg
                #return: a slice of data corresponding to the joints in leg with id leg_id
                #leg_id=1--->[data[1], data[2], data[3]]
                return data[self.leg_joints*(leg_id-1):self.leg_joints*(leg_id-1)+self.leg_joints]

                                
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
                        Q[3*i:3*i+3,j] = self.leg_inv_kine(T_i[:,j], i) # leg_id=1...se metto i non funziona. C'?? qualche incongruenza di segno sulle cinematiche
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

        def ik_stewart(self, displacement, orientation, F_b):
                """
                To be re-implemented when introducing multiple legs
                """
                rotation = eul.euler2mat(orientation[0],orientation[1],orientation[2],axes='szyx')
                #Anchor points of legs in the base frame
                A_b = np.tile(displacement,self.leg_num) + np.reshape(rotation.dot(self.leg_frame_b.T).T,18) 
                #Feet tip position in leg frames
                F_l = rotation.T.dot(np.reshape(F_b - A_b,(6,3)).T) 			
                #Mirror left side legs.
                F_l[0,self.leg_num//2:self.leg_num + 1] = -F_l[0,self.leg_num//2:self.leg_num + 1]
                F_l[1,self.leg_num//2:self.leg_num + 1] = -F_l[1,self.leg_num//2:self.leg_num + 1]
                return F_l.T


