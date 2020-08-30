#! /usr/bin/env python
"""treatment kinect information"""

import rospy
import argparse
import numpy as np
import cv2 as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

frame_c = []
frame_d = []
#global var to store object position in the image
# or the masks that delimitate the objects

#global var to store object position in the world

def color_callback(img_color):
    #convert Image into opencv format
    bridge = CvBridge()
    try:
        frame_c = bridge.imgmsg_to_cv2(img_color, "bgr8")
    except CvBridgeError, e:
        print e

    frame_c = np.array(frame_c, dtype=np.uint8)
    aux = True
    while(aux):
        cv.imshow("img_color",frame_c)
        key = cv.waitKey()
        if key == 'q':
            aux = False

    cv.destroyAllWindows()
    #info treatment
    #detection of objects
    #classification of objects

def depth_callback(img_depth):
    bridge = CvBridge()
    try:
        frame_d = bridge.imgmsg_to_cv2(img_depth)
    except CvBridgeError, e:
        print e

    frame_d = np.array(frame_d, dtype=np.float32)
    aux = True
    while(aux):
        cv.imshow("img_depth",frame_d)
        key = cv.waitKey()
        if key == 'q':
            aux = False

    cv.destroyAllWindows()
    #create mask of each detected and identified object
    #relate depth with real distance in the world


if __name__ == '__main__':
  try:
      rospy.init_node('detect_game')
      #create window to visualize result
      cv.namedWindow("img_color", cv.WINDOW_NORMAL)
      cv.namedWindow("img_depth", cv.WINDOW_NORMAL)
      #subscribing to image topics
      color_sub = rospy.Subscriber("/camera/color/image_raw", Image, color_callback)
      depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback)

      rospy.spin()


  except rospy.ROSInterruptException:
    print "program interrupted before completion"
