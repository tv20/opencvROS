#!/usr/bin/env python2
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
 
# Import the necessary libraries
import rospy # Python library for ROS
import roslib
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String
from std_msgs.msg import Int16
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from cv_basics.msg import box
import cv2 # OpenCV library
import numpy as np

path = "/home/tats/catkin_ws/src/cv_basics/src/buoy9.xml"
objectName = "BUOY"
color = (255,0,255)
pub = rospy.Publisher('object', box, queue_size=10)

def empty(a):
    pass

cv2.namedWindow("camera")
#cv2.createTrackbar("Scale","camera",400,1000,empty)
cv2.createTrackbar("Neig","camera",2,50,empty)
cv2.createTrackbar("Min Area","camera",0,100000,empty)

def findObjects(img):
    cascade = cv2.CascadeClassifier(path)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #scaleVal = 1 + (cv2.getTrackbarPos("Scale", "camera") / 1000)
    scaleVal = 1.05
    neig = cv2.getTrackbarPos("Neig", "camera")
    #neig = 1
    objects = cascade.detectMultiScale(gray,scaleVal,neig)  

    for (x,y,w,h) in objects:
        area = w*h
        minArea = cv2.getTrackbarPos("Min Area", "camera")
        if area > minArea:
            cv2.rectangle(img,(x,y),(x+w,y+h),color,3)
            cv2.putText(img,objectName,(x,y-5),cv2.FONT_HERSHEY_COMPLEX_SMALL,1,color,2)
            roi_color = img[y:y+h, x:x+w]
            msg = box()
            msg.object = objectName
            msg.x = x
            msg.y = y
            msg.xw = x+w
            msg.yh = y+h
            print(msg)
            pub.publish(msg) 
    return img


def callback(data):
    # Used to convert between ROS and OpenCV images
    br = CvBridge()
    # Output debugging information to the terminal
    #rospy.loginfo("receiving video frame")
    # Convert ROS Image message to OpenCV image
    img = br.imgmsg_to_cv2(data)
    imgRect = findObjects(img)

    cv2.imshow("camera", imgRect)
    cv2.waitKey(1)

def receive_message():
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('webcam_sub', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
    receive_message()