#!/usr/bin/env python

import RPi.GPIO as GPIO
import sys
import time
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray, Int32, Bool

GPIO.setmode(GPIO.BCM) #use GPIOnum names for pins

max_mux_freq = 1000000

#Configure GPIO pins
#GPIO 17, 22, 23, 24 writing (selector)
GPIO.setup(17, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(22, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(23, GPIO.OUT, initial = GPIO.LOW)
GPIO.setup(24, GPIO.OUT, initial = GPIO.LOW)
#GPIO 5, 6, 25 reading
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(25, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

#Create address table
mux_ch = {0: [0,0,0,0], 1: [0,0,0,1], 2: [0,0,1,0], 3:[0,0,1,1], 4:  [0,1,0,0], 5: [0,1,0,1], 6: [0,1,1,0], 7: [0,1,1,1], 8: [1,0,0,0]}

#Get arguments
sys.argv = rospy.myargv()
if len(sys.argv) is 2:
	rate = int(sys.argv[1])
	if rate <= 0 or rate >max_mux_freq: #bad argument value
		rate = 1 # default
else: #bad argument number
	rate = 1 #default rate is 1
rospy.loginfo('rate = %d', rate)

def read_loop():
    pub_left = rospy.Publisher('leakage/left_side', Int32MultiArray, queue_size=10)
    pub_right = rospy.Publisher('leakage/right_side', Int32MultiArray, queue_size=10)
    pub_cannister = rospy.Publisher('leakage/cannister', Int32MultiArray, queue_size=10)
    pub_overall  = rospy.Publisher('leakage', Bool, queue_size=10)
    freq = rospy.Rate(rate) #default 1
    left = Int32MultiArray()
    right = Int32MultiArray()
    cannister = Int32MultiArray()
    left.data = [0]*9
    right.data = [0]*9
    cannister.data = [0]*9
    while not rospy.is_shutdown():
        for i in range(0,len(left.data)):
		#select channel
		addr = mux_ch[i]
		GPIO.output(17, addr[3])
		GPIO.output(22, addr[2])
		GPIO.output(23, addr[1])
		GPIO.output(24, addr[0])
            	# Read the light sensor data
		left.data[i] = GPIO.input(6)
		right.data[i] = GPIO.input(5)
		cannister.data[i] = GPIO.input(25)

        #rospy.loginfo("Leakages left side: {}, Leakages right side: {}, Leakages cannister: {}".format(left.data, right.data, cannister.data))
        pub_left.publish(left)
        pub_right.publish(right)
        pub_cannister.publish(cannister)
        pub_overall.publish(any(left.data) or any(right.data) or any(cannister.data))
        freq.sleep()
    GPIO.cleanup()

if __name__ == '__main__':
    try:
	rospy.init_node('contact_sensing_node')
        read_loop()
    except rospy.ROSInterruptException:
        pass
