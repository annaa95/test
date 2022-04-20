#!/usr/bin/python

import spidev
import sys
import time
import os
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32

#Moving window low pass filter
window_size = 3
mw = 1.5*np.ones([window_size,6])
mw_tmp = np.zeros([window_size-1,6])

# Open SPI bus
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1000000

# Function to read SPI data from MCP3008 chip
# Channel must be an integer 0-7
def ReadChannel(channel):
  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  return data

# Function to convert data to voltage level,
# rounded to specified number of decimal places.
def ConvertVolts(data,places):
  volts = (data * 5) / float(1023)
  return round(volts,places)

# Function to convert data to current level
def ConvertAmpere(data,places):
  offset = 2.5 #V
  conversion = 0.066 #V/A
  volt = (data * 5) / float(1023)
  ampere = -(volt-offset)/conversion
  return round(ampere,places)

# Function to convert data to battery charge estimation
def ConvertBattery(data,places):
  vd = 4 #Voltage divider
  scaling = 1.2543 # scaling factor pinv()
  volt = (data * 5) / float(1023)
  battery = volt*vd*scaling
  return round(battery,places)

# Define sensor channels
nleg = 6
sys.argv = rospy.myargv() #remapping the argv to expected input
if len(sys.argv) is 2:
	rate = int(sys.argv[1])
	if rate <= 0 or rate >spi.max_speed_hz: #bad argument value
		rate = 100 # default
else: #bad argument number
	rate = 100 #default rate is 100
rospy.loginfo('rate = %d', rate)

def read_loop():
    pub_piezo = rospy.Publisher('adc/contacts', Float32MultiArray, queue_size=10)
    #pub_piezo_lowpass = rospy.Publisher('adc/contacts_lowpass', Float32MultiArray, queue_size = 10)
    pub_current = rospy.Publisher('adc/current', Float32, queue_size=10)
    pub_battery = rospy.Publisher('adc/battery_level', Float32, queue_size=10)
    freq = rospy.Rate(rate) #default 100
    volts_msg = Float32MultiArray()
    volts_mean_msg = Float32MultiArray()
    volts_msg.data = [0]*nleg
    volts_mean_msg.data = [0]*nleg
    current = 0
    battery_level = 0
    
    while not rospy.is_shutdown():

        #Read piezo
        for i,_ in enumerate(volts_msg.data):
            volts_msg.data[i] = ConvertVolts(ReadChannel(i),3)
        pub_piezo.publish(volts_msg)

        #shift mw
        #mw_tmp = mw[0:window_size-1,:]
        #mw[1:window_size,:] = mw_tmp
        
        #insert new read in free position
        #mw[0,:] = volts_msg.data

        #compute mean
        #for i,_ in enumerate(volts_msg.data):
          #volts_mean_msg.data[i] = np.mean(mw[:,i])
        #pub_piezo_lowpass.publish(volts_mean_msg)

        #Read current
        current = ConvertAmpere(ReadChannel(6), 3)
        pub_current.publish(current)

        #Read battery
        battery_level = ConvertBattery(ReadChannel(7), 3)
        pub_battery.publish(battery_level)

        #rospy.loginfo("Piezo voltage: {}V, Current: {}A, Battery: {}V".format(volts_msg.data,current, battery_level))
        freq.sleep()

  
if __name__ == '__main__':
    try:
	rospy.init_node('adc_driver_node')
        read_loop()
    except rospy.ROSInterruptException:
        pass

