#!/usr/bin/env python
# licence: me

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

def callback(data):
    try:
        cv_image = CvBridge().imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv2.imshow('camera', cv2.flip(cv_image,0))
        #rospy.loginfo('I can see')
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)

def frame_listener():
    rospy.init_node('frame_listener', anonymous=True)
#    rospy.Subscriber('FramePublisher', Image, callback)
    rospy.Subscriber('FramePublisherRight', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    frame_listener()
