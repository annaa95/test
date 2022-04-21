#!/usr/bin/env python
# licence: me

import rospy
from sensor_msgs.msg import Image
import sys

import cv2
from cv_bridge import CvBridge, CvBridgeError

camNo = int(sys.argv[1])

def frame_pub():
    print 'You selected', camNo, 'cameras to be published'
    if camNo == 1:
        pub = rospy.Publisher('CamPublisherHD', Image, queue_size=10)
        rospy.init_node('Cam_pub',anonymous=True)
        rate = rospy.Rate(1)
        cam = cv2.VideoCapture(2)
        cam.set(3,800)
        cam.set(4,600)
	w = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        h = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
	print w, h
        rospy.loginfo('Camera created')
        while not rospy.is_shutdown():
            res, img = cam.read()
            if res:
                img=cv2.flip(cv2.flip(img,0),1)
                frame_msg = CvBridge().cv2_to_imgmsg(img, 'bgr8')
                pub.publish(frame_msg)
                #rospy.loginfo('Image sent')
                rate.sleep()
    else:
            if camNo==2:
                # Left 
                pub1 = rospy.Publisher('CamPublisherLeft', Image, queue_size=10)
                rospy.init_node('Cams_pub',anonymous=True)
                cam1 = cv2.VideoCapture(0)
                cam1.set(3,1280)
                cam1.set(4,720)
                # Right camera
                pub2 = rospy.Publisher('CamPublisherRight', Image, queue_size=10)
                rate = rospy.Rate(25)
                cam2 = cv2.VideoCapture(2)
                cam2.set(3,1280)
                cam2.set(4,720)
                rospy.loginfo('Cameras created')
                 
                while not rospy.is_shutdown():
                    res1, img1 = cam1.read()             
                    res2, img2 = cam2.read()
                    img1=cv2.flip(cv2.flip(img1,0),1)   
                    img2=cv2.flip(cv2.flip(img2,0),1)
                    if res1:
                        frame_msg1 = CvBridge().cv2_to_imgmsg(img1, 'bgr8')
                        pub1.publish(frame_msg1)
                        if res2:
                            frame_msg2 = CvBridge().cv2_to_imgmsg(img2, 'bgr8')
                            pub2.publish(frame_msg2)
                        #rospy.loginfo('Image sent')
                        rate.sleep()
            else:
                print('Incorrect number of cameras requested')        

if __name__ == '__main__':
    try:
        frame_pub()
    except rospy.ROSInterruptException:
        pass
