#!/usr/bin/env python
#Author: Giacomo Picardi
#conversion servo cmd - gimble angle to be checked
import rospy
import math
from RPIO import PWM
from sensor_msgs.msg import Imu

servoPIN = 18
servo = PWM.Servo()

def callback(data):
    q1 = data.orientation.x
    q2 = data.orientation.y
    q3 = data.orientation.z
    q0 = data.orientation.w
    q_norm = math.sqrt(q0*q0+q1*q1+q2*q2+q3*q3)
    q1 = data.orientation.x/q_norm
    q2 = data.orientation.y/q_norm
    q3 = data.orientation.z/q_norm
    q0 = data.orientation.w/q_norm
    
    phi = math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
    theta = math.asin(2*(q0*q2-q3*q1))
    psi = math.atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))

    rospy.loginfo(rospy.get_caller_id() + "bank: %.2f, attitude: %.2f, heading: %.2f", phi*180/math.pi, theta*180/math.pi, psi*180/math.pi)
    
    pw = round(theta/math.pi*2*180)*10+1800
    servo.set_servo(servoPIN, pw)	
    
def  camera_stabilizer():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('camera_stabilizer', anonymous=True)

    rospy.Subscriber("imu_bosch/data", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    camera_stabilizer()
