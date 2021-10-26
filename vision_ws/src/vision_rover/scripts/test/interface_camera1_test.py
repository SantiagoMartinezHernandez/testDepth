#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
# from cv_bridge import CvBridge
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

    # img = CvBridge().imgmsg_to_cv2(msg)
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

    # if bandera: 
    #     bandera = not bandera
    #     print(type(msg))
    #     print("-----------------------")
    #     print(type(img))
    #     print("-----------------------")
    #     print(img.__repr__())
    #     print("-----------------------")
    #     print(img)

    showImage(img)

    # file = open("msgs.txt", 'w')
    # file.write(str(img)+"\n")
    # file.close()

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
