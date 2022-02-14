#!/usr/bin/env python

"""
Jose Cristobal Arroyo Castellanos

Para que el nodo funcione es necesario que se declare en el paquete local en el CMakeLists

find_package( OpenCV REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )

Al correr el nodo por separado debe tenerse en cuenta lo siguiente:

El codigo utiliza flags para crear un mismo tipo de nodo para ver las camaras. Para ejemplificar:

> python interface_camera_node.py /cam1 1 

Crea un nodo que publica en el topico '/cam1' y abre la camara con indice '1'
Sucede de igual forma con:

> rosrun vision_rover interface_camera_node.py /cam1 1

Para propositos especificos es necesario copiar este codigo y modificarlo, de modo que pueda ser
utilizado adecuadamente para lo que se necesite.

"""
print("[INFO] Starting Node")
import rospy
import cv2
import sys

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from utils import image_filters as imf

global send_image

# CONSTANTS PARAMETERS FOR CAMERAS BELOW
CAMERA_NAME = '/cam1'
CAMERA_INDEX = 0

if len(sys.argv) > 1:
    print(sys.argv)
    CAMERA_NAME = sys.argv[1]
    CAMERA_INDEX = int(sys.argv[2])

send_image = 0

def main():
    try:
        pub = rospy.Publisher(CAMERA_NAME, Image, queue_size=2)
        sub = rospy.Subscriber(CAMERA_NAME+'_signal', Float32, receive_signal)
        cap = cv2.VideoCapture(CAMERA_INDEX)
        bridge = CvBridge()
        rospy.loginfo("About to start stream. Waiting for signal")
    except Exception as e:
        rospy.logwarn(e)
    
    while(not rospy.is_shutdown()):
        try:
            ret, frame= cap.read()

            # DO ANYTHING WITH THE FRAME
        
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                raise KeyboardInterrupt

            if send_image != 0:
                if send_image == 1:
                    imgMsg = bridge.cv2_to_imgmsg(frame, "bgr8")
                elif send_image == 2:
                    imgMsg = bridge.cv2_to_imgmsg(imf.gray(frame), "mono8")
                elif send_image == 3:
                    imgMsg = bridge.cv2_to_imgmsg(imf.threshold(imf.gray(frame)), "mono8")
                elif send_image == 4:
                    imgMsg = bridge.cv2_to_imgmsg(imf.edges(frame), "bgr8")
                pub.publish(imgMsg)

                rospy.Rate(100).sleep()
            
        except KeyboardInterrupt as ki:
            cap.release()
            cv2.destroyAllWindows()
            break
        except TypeError as te:
            cap.release()
            cv2.destroyAllWindows()
            rospy.logerr(te.message)
            rospy.logerr("Cannot read frame from index {}".format(CAMERA_INDEX))
            rospy.logerr("Camera connection closed")
            return
    rospy.loginfo("Manual shutdown, closing stream")


def receive_signal(data):
    global send_image
    send_image = data.data
    if send_image == 0:
        rospy.loginfo("Image Stream for {} closed".format(CAMERA_NAME))
    elif send_image == 1:
        rospy.loginfo("Raw image Stream for {} opened".format(CAMERA_NAME))
    elif send_image == 2:
        rospy.loginfo("Gray image Stream for {} opened".format(CAMERA_NAME))
    elif send_image == 3:
        rospy.loginfo("Threshold image Stream for {} opened".format(CAMERA_NAME))
        
        

# rospy.init_node('cam1_node')
if __name__ == '__main__':
    rospy.init_node(CAMERA_NAME[1:]+"_streamer")
    main()