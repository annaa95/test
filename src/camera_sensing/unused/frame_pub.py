#!/usr/bin/env python
# licence: me

import rospy
from sensor_msgs.msg import Image
import sys

import cv2
from cv_bridge import CvBridge, CvBridgeError

print('Insert camera ID (0 - Left camera, 2 - Rigth camera')
camID = int(sys.argv[1])

def frame_pub():
    if camID == 0:
        pub = rospy.Publisher('FramePublisherLeft', Image, queue_size=10)
        rospy.init_node('frame_pubL',anonymous=True)
        rate = rospy.Rate(25)
        cam = cv2.VideoCapture(camID)
        rospy.loginfo('Image publishing Left')
        while not rospy.is_shutdown():
            res, img = cam.read()
            if res:
                frame_msg = CvBridge().cv2_to_imgmsg(img)
                pub.publish(frame_msg)
                #rospy.loginfo('Image sent')
                rate.sleep()
    else:
        pub = rospy.Publisher('FramePublisherRight', Image, queue_size=10)
        rospy.init_node('frame_pubR',anonymous=True)
        rate = rospy.Rate(25)
        cam = cv2.VideoCapture(camID)
        rospy.loginfo('Image publishing Right')
        while not rospy.is_shutdown():
            res, img = cam.read()
            if res:
                frame_msg = CvBridge().cv2_to_imgmsg(img)
                pub.publish(frame_msg)
                #rospy.loginfo('Image sent')
                rate.sleep()
            

if __name__ == '__main__':
    try:
        frame_pub()
    except rospy.ROSInterruptException:
        pass
