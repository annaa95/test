#!/usr/bin/env python
# licence: me

import rospy
from sensor_msgs.msg import Image, CompressedImage # 04/03/2020 Mcal
from std_srvs.srv import Empty, EmptyResponse
from silver_msgs.srv import VideoCapture, VideoCaptureResponse
import sys

import cv2
from cv_bridge import CvBridge, CvBridgeError

import time

camNo = int(sys.argv[1])
im_scale = 3 #for LM, old value im_scale=3

global isRecording
isRecording = False

def rec_video(req):
    global isRecording
    if isRecording:
        isRecording = False
        print "stop recording"
        return VideoCaptureResponse(True, "Stop recording")
    else:
        isRecording = True
        print "start recording"
        return VideoCaptureResponse(True, "Start recording")

# create services
record = rospy.Service('VideoRecording', VideoCapture, rec_video) #MMM

def frame_pub():
    imgNo = 0
    print 'You selected', camNo, 'cameras to be published'
    if camNo == 1:
        #pub = rospy.Publisher('CamPublisher', Image, queue_size=10)
        pub = rospy.Publisher('CamPublisher', CompressedImage, queue_size=5)
        rospy.init_node('Cam_pub',anonymous=True)
        rate = rospy.Rate(30) #for LM, old value rospy.Rate(15)
        cam = cv2.VideoCapture(2)
        cam.set(6, cv2.VideoWriter_fourcc('M','J','P','G')) #obbligatorio per avere il massimo di risoluzione
        cam.set(cv2.CAP_PROP_FPS,30)
        #cam.set(cv2.CAP_PROP_AUTO_EXPOSURE,0)
        print cam.get(cv2.CAP_PROP_AUTO_EXPOSURE)
        cam.set(3,800)#cam.set(3,1920/im_scale)#1920
        cam.set(4,600)#cam.set(4,1080/im_scale)#1080
        rospy.loginfo('Camera created')
        while not rospy.is_shutdown():
            res, img = cam.read()
            if res:
                img=cv2.flip(cv2.flip(img,0),1)
                #imgR = cv2.resize(img, (1920/im_scale, 1080/im_scale), interpolation = cv2.INTER_AREA)
                ##frame_msg = CvBridge().cv2_to_imgmsg(cv2.resize(img, (320, 240), interpolation = cv2.INTER_AREA), 'bgr8')
                frame_msg = CvBridge().cv2_to_compressed_imgmsg(img, 'jpeg') #04/03/2020 Mcal
                pub.publish(frame_msg)
                if isRecording:
                    imgNo=imgNo+1
                    imName = '/home/ubuntu/Scrivania/videoSilver/test_img' + time.strftime("%Y-%m-%d-%H-%M-%S",time.gmtime(time.time())) + '.jpeg'
                    cv2.imwrite(imName,img)
                    #rospy.loginfo('Image sent')
                rate.sleep()
    else:
            if camNo==2:
                # Left 
                pub1 = rospy.Publisher('CamPublisherLeft', Image, queue_size=10)
                rospy.init_node('Cams_pub',anonymous=True)
                cam1 = cv2.VideoCapture(0)
                cam1.set(3,800)
                cam1.set(4,600)
                # Right camera
                pub2 = rospy.Publisher('CamPublisherRight', Image, queue_size=10)
                rate = rospy.Rate(25)
                cam2 = cv2.VideoCapture(2)
                cam2.set(3,800)
                cam2.set(4,600)
                rospy.loginfo('Cameras created')
                 
                while not rospy.is_shutdown():
                    res1, img1 = cam1.read()             
                    res2, img2 = cam2.read()
                    img1=cv2.flip(cv2.flip(img1,0),1)   
                    img2=cv2.flip(cv2.flip(img2,0),1)
                    if res1:
                        #img1small = cv2.resize( img1, (320, 240) )
                        img1small  = img1
                        frame_msg1 = CvBridge().cv2_to_imgmsg(img1small, 'bgr8')
                        pub1.publish(frame_msg1)
                        imgNo=imgNo+1
                        if isRecording and imgNo%10 == 0:
                            imName = '/home/ubuntu/Scrivania/videoSilver/img' + time.strftime("%Y-%m-%d-%H-%M-%S",time.gmtime(time.time())) + 'c1_'+ str(imgNo) + '.png'
                            cv2.imwrite(imName,img1)
                        if res2:
                            ## to be activated if I want to see 2 cameras
                            #img2small = cv2.resize( img2, (320, 240) )
                            img2small = img2
                            frame_msg2 = CvBridge().cv2_to_imgmsg(img2small, 'bgr8')
                            pub2.publish(frame_msg2) 
                            if isRecording and imgNo%10 == 0:
                                imName = '/home/ubuntu/Scrivania/videoSilver/img' + time.strftime("%Y-%m-%d-%H-%M-%S",time.gmtime(time.time())) + 'c2_'+ str(imgNo) + '.png'
                                cv2.imwrite(imName,img2)
                        #rospy.loginfo('Image sent')
                        rate.sleep()
            else:
                print('Incorrect number of cameras requested')        

if __name__ == '__main__':
    try:
        frame_pub()
    except rospy.ROSInterruptException:
	if camNo==1:
		cam.release()
	elif camNo==2:
		cam1.release()
		cam2.release()
        pass
