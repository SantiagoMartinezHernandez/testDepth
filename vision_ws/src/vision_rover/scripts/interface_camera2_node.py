#!/usr/bin/env python

"""
Jose Cristobal Arroyo Castellanos

Para que el nodo funcione es necesario que se declare en el paquete local en el CMakeLists

find_package( OpenCV REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )

"""
print("[INFO] Starting Node")
import rospy
import cv2
from utils.depth_module import *

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32   

global send_image

# CONSTANTS PARAMETERS FOR CAMERAS BELOW
CAMERA_NAME = '/cam2'
CAMERA_INDEX = 1

send_image = False

def main():
    pub = rospy.Publisher(CAMERA_NAME, Image, queue_size=10)
    sub = rospy.Subscriber(CAMERA_NAME+'_signal', Float32, receive_signal)
    cap = DepthCamera()
    bridge = CvBridge()
    print("[INFO] About to start stream. Waiting for signal")
    while(1):
        _, depth_frame, frame, colorized_depth = cap.read()
        # DO ANYTHING WITH THE FRAME

        if send_image:
            imgMsg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(imgMsg)
            rospy.Rate(100).sleep()

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cap.release()
            cv2.destroyAllWindows()
            break

def receive_signal(data):
    global send_image

    if data == Float32(1):
        send_image = True
        print("[INFO] Image Stream for {} opened".format(CAMERA_NAME))
    else:
        send_image = False
        print("[INFO] Image Stream for {} closed".format(CAMERA_NAME))
        

# rospy.init_node('cam1_node')
if __name__ == '__main__':
    rospy.init_node(CAMERA_NAME[1:]+"_streamer")
    main()