#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2

import numpy as np

global bandera

bandera = True

# CONSTANTS PARAMETERS FOR CAMERAS BELOW
CAMERA_NAME = '/cam1'

def start_node():
    global pub
    rospy.init_node(CAMERA_NAME[1:]+'_subscriber')
    rospy.loginfo(CAMERA_NAME[1:]+'_subscriber started')
    rospy.Subscriber(CAMERA_NAME, Image , process_image)
    pub = rospy.Publisher(CAMERA_NAME+"_signal", Float32, queue_size=10)
    rospy.spin()

def process_image(msg):
    global pub
    global bandera

    img = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    showImage(img)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cv2.destroyAllWindows()    
        
def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(10)
        
    
if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
