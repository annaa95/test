#!/usr/bin/python

import ms5837
import time
import sys
import rospy
from std_msgs.msg import Float32

sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)
max_rate = 1000000

sys.argv = rospy.myargv()
if len(sys.argv) is 2:
	rate = int(sys.argv[1])
	if rate <= 0 or rate >max_rate: #bad argument value
		rate = 100 # default
else: #bad argument number
	rate = 100 #default rate is 100
rospy.loginfo('rate = %d', rate)

# We must initialize the sensor before reading it
if not sensor.init():
        print "Sensor could not be initialized"
        exit(1)

def read_loop():
    pub = rospy.Publisher('ms5837/pressure', Float32, queue_size=10)
    pub2 = rospy.Publisher('ms5837/temperature', Float32, queue_size=10)
    freq = rospy.Rate(rate) #default 100
    pressure = 0
    temperature = 0
    while not rospy.is_shutdown():
	if sensor.read():
	        pressure = sensor.pressure() #mbar
		temperature = sensor.temperature() #celsius
	        #rospy.loginfo("Pressure: {}mbar - Temperature: {}C".format(pressure, temperature))
	        pub.publish(pressure)
		pub2.publish(temperature)
       		freq.sleep()
	else:
		print "Sensor read failed!"
                exit(1)

if __name__ == '__main__':
    try:
	rospy.init_node('ms5837_node')
        read_loop()
    except rospy.ROSInterruptException:
        pass
