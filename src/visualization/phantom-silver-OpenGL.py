#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Marce's code for visualization
# It uses the pygame lib for windows, OpenGL for visualization, and numpy for calculations
# This version uses the kinematics of the actual robot, with minor adaptations (ATT: the kine is taken from robot.py,
# and it is NOT taken from pahntom_visualizer.py)

# V0. 24 Marzo 2020 copyright: Marcello Calisti

import pygame
from pygame.locals import *
import numpy as np
import numpy.matlib
import math
from OpenGL.GL import *
from OpenGL.GLU import *

import rospy
from silver_msgs.msg import PhantomVisualization

import threading
import time
import signal
import os, sys

fileDir = os.path.dirname(os.path.abspath(__file__))
parentDir = os.path.dirname(fileDir)
path = sys.path
path.append(parentDir)

should_quit = False
def handler(signum, frame):
    global should_quit
    should_quit = True

LegE = np.array([[0,1],[1,2]])
axisColor = np.array([0.2, 0.2, 0.2])
legColor = np.array([1.0, 0.75, 0.0])
bodyColor = np.array([1.0, 0.75, 0.0])
nodeColor = np.array([1.0, 0.75, 0.0])

#Structural characteristics of the robot.
leg_num = 6				#Number of legs
leg_joints = 3			#Number of joints in a leg
joint_num = leg_num*leg_joints	#Number of joints in the robot
l0 = 0.08 					#Length of leg segments in cm
l1 = 0.07
l2 = 0.30
l3 = 0.323 					#26 without foot
body_length = 0.40				#Dimensions of body in m
body_width = 0.55
body_height = 0.35

BodyV = np.array([[body_length/2,-body_height/2,-body_width/2],
		[body_length/2,body_height/2,-body_width/2],
		[-body_length/2,body_height/2,-body_width/2],
		[-body_length/2,-body_height/2,-body_width/2],
		[body_length/2,-body_height/2,body_width/2],
		[body_length/2,body_height/2,body_width/2],
		[-body_length/2,-body_height/2,body_width/2],
		[-body_length/2,body_height/2,body_width/2]])

BodyE = np.array([[0,1],[0,3],[0,4],[2,1],[2,3],[2,7],[6,3],[6,4],[6,7],[5,1],[5,4],[5,7]])

#Position of leg frames within the base frame. #MATLIB visuazation frame
leg_frame_b = np.zeros([leg_num,leg_joints])
leg_frame_b = np.zeros([leg_num,leg_joints])
leg_frame_b[0,:] = np.array([body_length/2 , body_width/2  , -body_height/2])
leg_frame_b[1,:] = np.array([body_length/2 , 0             , -body_height/2])
leg_frame_b[2,:] = np.array([body_length/2 , -body_width/2 , -body_height/2])
leg_frame_b[3,:] = np.array([-body_length/2, body_width/2  , -body_height/2])
leg_frame_b[4,:] = np.array([-body_length/2, 0             , -body_height/2])
leg_frame_b[5,:] = np.array([-body_length/2, -body_width/2 , -body_height/2])

#Leg frames mirroring.
l_pos = np.array([1, 1, -1, 1, 1, -1])

global sPos, ePos

sPos = np.array([0,0])
ePos = np.array([0,0])

# Input: joint angles and leg id. Output: (x,y,z) of the foot. Please not that (x,y,z) in leg-Frame
# is different from World-Frame:
def kine_leg( q, leg_id):
        if leg_id <= 2:
                q1 = q[0]
        else:
                q1 = -q[0]
        q2 = q[1]
        q3 = q[2]

        x = (l0 + l2*np.cos(q2) + l3*np.cos(q2-q3))*np.cos(q1) - l_pos[leg_id]*l1*np.sin(q1)
        y = (l0 + l2*np.cos(q2) + l3*np.cos(q2-q3))*np.sin(q1) + l_pos[leg_id]*l1*np.cos(q1)
        z = l2*np.sin(q2) + l3*np.sin(q2-q3)

        return np.array([np.asscalar(x), np.asscalar(y), np.asscalar(z)])

# Input: joint angles and leg id. Output: (x,y,z) of the knee
def kine_knee( q, leg_id):
        if leg_id <= 2:
                q1 = q[0]
        else:
                q1 = -q[0]
        q2 = q[1]

        x = (l0 + l2*np.cos(q2))*np.cos(q1) - l_pos[leg_id]*l1*np.sin(q1)
        y = (l0 + l2*np.cos(q2))*np.sin(q1) + l_pos[leg_id]*l1*np.cos(q1)
        z = l2*np.sin(q2)

        return np.array([np.asscalar(x), np.asscalar(y), np.asscalar(z)])

# Input: joint angles and leg id. Output: (x,y,z) of the second motor (femur)
def kine_femurjoint( q, leg_id):
        if leg_id <= 2:
                q1 = q[0]
        else:
                q1 = -q[0]

        x = l0*np.cos(q1) - l_pos[leg_id]*l1*np.sin(q1)
        y = l0*np.sin(q1) + l_pos[leg_id]*l1*np.cos(q1)
        z = 0

        return np.array([np.asscalar(x), np.asscalar(y), z])

def kine_lbend( q, leg_id):
        if leg_id <= 2:
                q1 = q[0]
        else:
                q1 = -q[0]

        x = - l_pos[leg_id]*l1*np.sin(q1)
        y = l_pos[leg_id]*l1*np.cos(q1)
        z = 0

        return np.array([np.asscalar(x), np.asscalar(y), z])

def drawBody():
	glColor3f(bodyColor[0],bodyColor[1],bodyColor[2])
	glLineWidth(1.5)
	glBegin(GL_LINES)
	for edge in BodyE:
		for vertex in edge:
			glVertex3fv(BodyV[vertex])
	glEnd()

def drawPoints(vertVec):
	glColor3f(nodeColor[0],nodeColor[1],nodeColor[2])
	glPointSize(5.0)
	glBegin(GL_POINTS)
	for p in vertVec:
		glVertex3fv(p)
	glEnd()

def drawLegs(vertVec):
    glColor3f(legColor[0],legColor[1],legColor[2])
    glLineWidth(1.5)
    glBegin(GL_LINES)
    for edge in LegE:
        for vertex in edge:
            glVertex3fv(vertVec[vertex])
    glEnd()

def drawAxis():
	grid = 2.0
	glLineWidth(0.5)
	glColor3f(axisColor[0],axisColor[1],axisColor[2])
	glBegin(GL_LINES)
	for i in range(-2,3,2):
		for j in range(-2,3,2):
			glVertex3f(-1.0,i/grid,j/grid)
			glVertex3f(1.0,i/grid,j/grid)
			glVertex3f(i/grid,-1.0,j/grid)
			glVertex3f(i/grid,1.0,j/grid)
			glVertex3f(i/grid,j/grid,-1.0)
			glVertex3f(i/grid,j/grid,1.0)
	glEnd()

def drawDome(center):
    radius = 0.1
    npoints = 50
    glPointSize(1.5)
    glBegin(GL_LINES)
    for i in range(0,npoints):
        glVertex3f(center[0]+radius*np.cos(i*2*np.pi/npoints),center[1]+body_height/4+radius*np.sin(i*2*np.pi/npoints),center[2]+body_width/2)
        glVertex3f(center[0]+radius*np.cos((i+1)*2*np.pi/npoints),center[1]+body_height/4+radius*np.sin((i+1)*2*np.pi/npoints),center[2]+body_width/2)
        glVertex3f(center[0]+radius*np.cos(i*np.pi/npoints),center[1]+body_height/4,center[2]+body_width/2+radius*np.sin(i*np.pi/npoints))
        glVertex3f(center[0]+radius*np.cos((i+1)*np.pi/npoints),center[1]+body_height/4,center[2]+body_width/2+radius*np.sin((i+1)*np.pi/npoints))
        glVertex3f(center[0],center[1]+body_height/4+radius*np.cos(i*np.pi/npoints),center[2]+body_width/2+radius*np.sin(i*np.pi/npoints))
        glVertex3f(center[0],center[1]+body_height/4+radius*np.cos((i+1)*np.pi/npoints),center[2]+body_width/2+radius*np.sin((i+1)*np.pi/npoints))
    glEnd()

def winClosure():
	print('\nClosing visualization...')
	pygame.quit()
	quit()

def mouseMove(event):
	global sPos, ePos

	if event.type == pygame.MOUSEBUTTONDOWN and event.button == 4: # wheel rolled up
		glScaled(1.05, 1.05, 1.05)
	elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 5: # wheel rolled down
		glScaled(0.95, 0.95, 0.95)
	elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1: # left-button touch_down
		sPos = pygame.mouse.get_pos()
	elif event.type == pygame.MOUSEBUTTONUP and event.button == 1: # left-button touch_down
		ePos = pygame.mouse.get_pos()
		diffPos = np.subtract(ePos,sPos)
		if numpy.linalg.norm(diffPos)>25.0:
			if abs(diffPos[0])>abs(diffPos[1]):
				glRotatef(diffPos[0]/25.0, 0.0, 1.0, 0.0)
			else:
				glRotatef(diffPos[1]/25.0, 1.0, 0.0, 0.0)
		#glRotatef(-1.0, 0.0, 1.0, 0.0)

# ----------- window creation and inizialization

class OpenGLvisualizer:

    def __init__(self):
        #internal variables and initializations
        self.res = np.array([800,600])
        self.joints = np.zeros(18)
        self.F_b = np.zeros([leg_joints,leg_num])
        self.K_b = np.zeros([leg_joints,leg_num])
        self.FJ_b = np.zeros([leg_joints,leg_num])
        # subscribe to topics
        rospy.Subscriber('robot_angles_and_status',PhantomVisualization,self.getRobot, queue_size=1)

        pygame.init()
        try:
            ico = pygame.image.load(parentDir+'/visualization/ico/SV.png')
            pygame.display.set_icon(ico)
            pass
        except pygame.error as message:
            try:
                ico = pygame.image.load('SV.png')
                pygame.display.set_icon(ico)
                pass
            except pygame.error as message:
                print('No icon found for Silver Visualizers')
            #raise SystemExit(message)
        pygame.display.set_mode(self.res, DOUBLEBUF|OPENGL)
        pygame.display.set_caption('Phantom-SILVER')

        gluPerspective(45, (self.res[0]/self.res[1]), 0.1, 50.0)

        glTranslatef(0.0,0.25, -2.5)
        glRotatef(25.0, 1.0, 0.0, 0.0)
        glRotatef(45.0, 0.0, 1.0, 0.0)

        drawAxis()

    def ros_spin(self):
        rospy.spin()
        '''
        def __del__(self):
        print('Closing the PyGame windows')
        #prevhand(signal.SIGINT, None)
        #pygame.quit()
        #quit()
        '''
    # ------------ retrieving of message data and drawing
    def getRobot(self,data):
        # ---------
        #Create the joint angle matrix
        #joints = np.pi*1/180*np.array([45,70,150,0,70,150,-45,70,150,-45,70,150,0,70,150,45,70,150])
        #joints_status = np.ones(18)
        #
        self.joints =  np.array(data.motor_angles)
        self.joints_status = np.array(data.motor_status)

        joint_pos = np.matrix(self.joints)
        # in this way the matrix joint_pos (3,6) has in each row the six coxa , then the six femur and eventually the tibia angles
        joint_pos = joint_pos.reshape((leg_num,leg_joints)).transpose()

        #Create storing variables for interesting point positioning. (Matrixes (3,6), as in joint_pos)
        F_l = np.zeros(joint_pos.shape)  #feet
        K_l = np.zeros(joint_pos.shape)  #knees
        FJ_l = np.zeros(joint_pos.shape) #femur joints

        #Get feet, knee, femur joint and lbend positions in leg frame.
        for leg_id in range(0,leg_num):
            F_l[:,leg_id] = kine_leg(joint_pos[:,leg_id],leg_id)
            K_l[:,leg_id] = kine_knee(joint_pos[:,leg_id],leg_id)
            FJ_l[:,leg_id] = kine_femurjoint(joint_pos[:,leg_id],leg_id)

        #Mirroring left side legs.
        F_l[0,leg_num//2:leg_num + 1] = -F_l[0,leg_num//2:leg_num + 1]
        K_l[0,leg_num//2:leg_num + 1] = -K_l[0,leg_num//2:leg_num + 1]
        FJ_l[0,leg_num//2:leg_num + 1] = -FJ_l[0,leg_num//2:leg_num + 1]

        #Get feet-tip, knee and shoulder position in base frame. # NOTE: rot.dot(mat) = rot X mat
        #leg_frame_b.transpose()+F_l is the translation of the leg frame into world frame. I guess that
        #displacement and rotation are the matrix for visualization purposes only (?)
        self.F_b = leg_frame_b.transpose() + F_l
        self.K_b = leg_frame_b.transpose() + K_l
        self.FJ_b = leg_frame_b.transpose() + FJ_l
        # ---------

    def loop(self):
        rosthread = threading.Thread(target=self.ros_spin)
        rosthread.deamon = True
        rosthread.start()
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    print('Invoking manually ROS node kill')
                    prevhand(signal.SIGINT, None)
                    print('Closing the PyGame windows')
                    pygame.quit()
                    quit()

                mouseMove(event)

            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
            drawAxis()
            drawBody()
            drawDome(np.array([0,0,0]))
            for leg_id in range(0,leg_num):
                # y and z inverted
                drawPoints(np.array([ [self.FJ_b[0,leg_id],self.FJ_b[2,leg_id],self.FJ_b[1,leg_id]], [self.K_b[0,leg_id],self.K_b[2,leg_id],self.K_b[1,leg_id]], [self.F_b[0,leg_id],self.F_b[2,leg_id],self.F_b[1,leg_id]] ]))
                drawLegs(np.array([ [self.FJ_b[0,leg_id],self.FJ_b[2,leg_id],self.FJ_b[1,leg_id]], [self.K_b[0,leg_id],self.K_b[2,leg_id],self.K_b[1,leg_id]], [self.F_b[0,leg_id],self.F_b[2,leg_id],self.F_b[1,leg_id]] ]))

            #glRotatef(-1.0, 0.0, 1.0, 0.0)
            pygame.display.flip()
            pygame.time.wait(100)

        #self.__del__() # thre is a not clear behavior on the shutting down. With this combination, it seems working properly.

# --- main run of the code
if __name__ == '__main__':

	rospy.init_node('phantom')
	# prevhand is the handler set by the ROS runtime
	prevhand = signal.signal(signal.SIGINT, handler)
	OGL = OpenGLvisualizer()

	OGL.loop()

	print('Normal exit')
	# calls ROS signal handler manually
	# this is to stop the rospy.spin thread
	prevhand(signal.SIGINT, None)
