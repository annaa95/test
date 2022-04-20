#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import sys, os
import numpy as np
import numpy.matlib
import math
# Packages for 3D visualization.
from mpl_toolkits import mplot3d
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from matplotlib import colors

import rospy
from silver_msgs.msg import PhantomVisualization

class Visualizer:
	def __init__(self, leg_num, leg_joints):
		#Structural characteristics of the robot.
                self.leg_num = leg_num				#Number of legs
                self.leg_joints = leg_joints			#Number of joints in a leg
                self.joint_num = self.leg_num*self.leg_joints	#Number of joints in the robot
                self.l0 = 8 					#Length of leg segments in cm
                self.l1 = 7
                self.l2 = 30
                self.l3 = 32.3 					#26 without foot
                self.body_length = 40				#Dimensions of body in cm
                self.body_width = 55

		#Position of leg frames within the base frame.
                self.leg_frame_b = np.zeros([self.leg_num,self.leg_joints])
                self.leg_frame_b[0,:] = np.array([self.body_length/2, self.body_width/2, 0])
                self.leg_frame_b[1,:] = np.array([self.body_length/2, 0, 0])
                self.leg_frame_b[2,:] = np.array([self.body_length/2, -self.body_width/2, 0])
                self.leg_frame_b[3,:] = np.array([-self.body_length/2, self.body_width/2, 0])
                self.leg_frame_b[4,:] = np.array([-self.body_length/2, 0, 0])
                self.leg_frame_b[5,:] = np.array([-self.body_length/2, -self.body_width/2, 0])
                #Leg frames mirroring.
                self.l_pos = np.array([1, 1, -1, 1, 1, -1])
		
		plt.ion()

	def show_robot(self, req):
                #Catch information from the service message.
            	joints = np.array(req.motor_angles)
            	joints_status = np.array(req.motor_status)
                print(joints)
                print(joints_status)

                displacement = np.array([[0],[0],[50]])
		rotation = np.matrix([[1,0,0],
				      [0,1,0],
				      [0,0,1]])

                #Create new axis.
                scene = plt.figure(1)
		cur_axis = scene.axes
		if len(cur_axis) == 0:
			scene_ax = scene.add_subplot(111, projection='3d')
		else:
			scene_ax = cur_axis[0]
		plt.cla()
                scene_ax.set_xlabel('X')
                scene_ax.set_ylabel('Y')
                scene_ax.set_zlabel('Z')

                #Create the joint angle matrixself.
                joint_pos = np.matrix(joints)
                joint_pos = joint_pos.reshape((self.leg_num,self.leg_joints)).transpose()

                #Create storing variables for interesting point positioning.
                F_l = np.zeros(joint_pos.shape)  #feet
                K_l = np.zeros(joint_pos.shape)  #knees
                FJ_l = np.zeros(joint_pos.shape) #femur joints
                LB_l = np.zeros(joint_pos.shape) #L bend
                A_l = np.zeros(joint_pos.shape)  #leg attachment point
                B_l = np.zeros(joint_pos.shape)  #upper rectangle of body
                B_l[2,:] = 20;

                #Get feet, knee, femur joint and lbend positions in leg frame.
                for leg_id in range(0,self.leg_num):
                        F_l[:,leg_id] = self.kine_leg(joint_pos[:,leg_id],leg_id)
                        K_l[:,leg_id] = self.kine_knee(joint_pos[:,leg_id],leg_id)
                        FJ_l[:,leg_id] = self.kine_femurjoint(joint_pos[:,leg_id],leg_id)
                        LB_l[:,leg_id] = self.kine_lbend(joint_pos[:,leg_id],leg_id)

                #Mirroring left side legs.
                F_l[0,self.leg_num//2:self.leg_num + 1] = -F_l[0,self.leg_num//2:self.leg_num + 1]
                K_l[0,self.leg_num//2:self.leg_num + 1] = -K_l[0,self.leg_num//2:self.leg_num + 1]
                FJ_l[0,self.leg_num//2:self.leg_num + 1] = -FJ_l[0,self.leg_num//2:self.leg_num + 1]
                LB_l[0,self.leg_num//2:self.leg_num + 1] = -LB_l[0,self.leg_num//2:self.leg_num + 1]

                #Get feet-tip, knee and shoulder position in base frame.
                F_b = displacement + rotation.dot(self.leg_frame_b.transpose() + F_l)
                K_b = displacement + rotation.dot(self.leg_frame_b.transpose() + K_l)
                FJ_b = displacement + rotation.dot(self.leg_frame_b.transpose() + FJ_l)
                LB_b = displacement + rotation.dot(self.leg_frame_b.transpose() + LB_l)
                A_b = displacement + rotation.dot(self.leg_frame_b.transpose() + A_l)
                B_b = displacement + rotation.dot(self.leg_frame_b.transpose() + B_l)

                #Plot the body.
                verts = [[A_b[:,0],A_b[:,2],A_b[:,-1],A_b[:,3]],
                	 [B_b[:,0],B_b[:,2],B_b[:,-1],B_b[:,3]],
                	 [A_b[:,0],A_b[:,2],B_b[:,2],B_b[:,0]],
                	 [A_b[:,2],A_b[:,-1],B_b[:,-1],B_b[:,2]],
                         [A_b[:,-1],A_b[:,3],B_b[:,3],B_b[:,-1]],
                	 [A_b[:,0],A_b[:,3],B_b[:,3],B_b[:,0]]]
                scene_ax.add_collection3d(Poly3DCollection(verts, facecolors='silver', linewidths=1, edgecolors='darkslategrey', alpha=0.5))

                #Plot legs.
                for leg_id in range(0,self.leg_num):
                        if joints_status[self.leg_joints*leg_id] == 0:
                            color = 'red'
                        else:
                            color = 'green'
                        scene_ax.scatter(A_b[0,leg_id],A_b[1,leg_id],A_b[2,leg_id],s=5,c=color)
                        scene_ax.plot([A_b[0,leg_id],LB_b[0,leg_id]],[A_b[1,leg_id],LB_b[1,leg_id]],[A_b[2,leg_id],LB_b[2,leg_id]],color='indigo',linewidth=2)
                        scene_ax.plot([LB_b[0,leg_id],FJ_b[0,leg_id]],[LB_b[1,leg_id],FJ_b[1,leg_id]],[LB_b[2,leg_id],FJ_b[2,leg_id]],color='darkgreen',linewidth=2)
                        if joints_status[self.leg_joints*leg_id + 1] == 0:
                            color = 'red'
                        else:
                            color = 'green'
                        scene_ax.scatter(FJ_b[0,leg_id],FJ_b[1,leg_id],FJ_b[2,leg_id],s=5,c=color)
                        scene_ax.plot([FJ_b[0,leg_id],K_b[0,leg_id]],[FJ_b[1,leg_id],K_b[1,leg_id]],[FJ_b[2,leg_id],K_b[2,leg_id]],color='black',linewidth=2)
                        if joints_status[self.leg_joints*leg_id + 2] == 0:
                            color = 'red'
                        else:
                            color = 'green'
                        scene_ax.scatter(K_b[0,leg_id],K_b[1,leg_id],K_b[2,leg_id],s=5,c=color)
                        scene_ax.plot([K_b[0,leg_id],F_b[0,leg_id]],[K_b[1,leg_id],F_b[1,leg_id]],[K_b[2,leg_id],F_b[2,leg_id]],color='slategrey',linewidth=2)
                        scene_ax.scatter(F_b[0,leg_id],F_b[1,leg_id],F_b[2,leg_id],c='k')

                scene_ax.xaxis.set_pane_color((0.12549019607843137, 0.6980392156862745, 0.6666666666666666, 0.5))
                scene_ax.yaxis.set_pane_color((0.12549019607843137, 0.6980392156862745, 0.6666666666666666, 0.5))
                scene_ax.zaxis.set_pane_color((0.8549019607843137, 0.6470588235294118, 0.12549019607843137, 0.5))
                scene_ax.set(xlim=(-100, 100), ylim=(-100, 100), zlim =(0, 100))
		
		
                #plt.draw()
		#plt.pause(0.01)
		plt.gcf().canvas.flush_events()
		plt.show(block = False)
		plt.show(block = False)

        def kine_leg(self, q, leg_id):
                if leg_id <= 2:
                        q1 = q[0]
                else:
                        q1 = -q[0]
                q2 = q[1]
                q3 = q[2]

                x = (self.l0 + self.l2*np.cos(q2) + self.l3*np.cos(q2-q3))*np.cos(q1) - self.l_pos[leg_id]*self.l1*np.sin(q1)
                y = (self.l0 + self.l2*np.cos(q2) + self.l3*np.cos(q2-q3))*np.sin(q1) + self.l_pos[leg_id]*self.l1*np.cos(q1)
                z = self.l2*np.sin(q2) + self.l3*np.sin(q2-q3)

                return np.array([np.asscalar(x), np.asscalar(y), np.asscalar(z)])

        def kine_knee(self, q, leg_id):
                if leg_id <= 2:
                        q1 = q[0]
                else:
                        q1 = -q[0]
                q2 = q[1]

                x = (self.l0 + self.l2*np.cos(q2))*np.cos(q1) - self.l_pos[leg_id]*self.l1*np.sin(q1)
                y = (self.l0 + self.l2*np.cos(q2))*np.sin(q1) + self.l_pos[leg_id]*self.l1*np.cos(q1)
                z = self.l2*np.sin(q2)

                return np.array([np.asscalar(x), np.asscalar(y), np.asscalar(z)])

        def kine_femurjoint(self, q, leg_id):
                if leg_id <= 2:
                        q1 = q[0]
                else:
                        q1 = -q[0]

                x = self.l0*np.cos(q1) - self.l_pos[leg_id]*self.l1*np.sin(q1)
                y = self.l0*np.sin(q1) + self.l_pos[leg_id]*self.l1*np.cos(q1)
                z = 0

                return np.array([np.asscalar(x), np.asscalar(y), z])

        def kine_lbend(self, q, leg_id):
                if leg_id <= 2:
                        q1 = q[0]
                else:
                        q1 = -q[0]

                x = - self.l_pos[leg_id]*self.l1*np.sin(q1)
                y = self.l_pos[leg_id]*self.l1*np.cos(q1)
                z = 0

                return np.array([np.asscalar(x), np.asscalar(y), z])

phantom = Visualizer(6,3)

rospy.init_node('phantom')
rospy.Subscriber('robot_angles_and_status',PhantomVisualization,phantom.show_robot)
rospy.spin()
