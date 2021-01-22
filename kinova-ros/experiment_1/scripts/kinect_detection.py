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
font = cv.FONT_HERSHEY_COMPLEX
#global var to store object position in the image
# or the masks that delimitate the objects

#global var to store object position in the world

def nothing(x):
    pass

def color_callback(img_color):
    #convert Image into opencv format
    bridge = CvBridge()
    try:
        frame_c = bridge.imgmsg_to_cv2(img_color, "bgr8")
    except CvBridgeError, e:
        print e


    frame_c = np.array(frame_c, dtype=np.uint8)
    # Convert to greyscale
    grey = cv.cvtColor(frame_c, cv.COLOR_BGR2GRAY)
    # Blur the image
    #grey = cv.blur(grey, (7, 7))# in simulation there is no noise yet
    # Compute edges using the Canny edge filter
    edges = cv.Canny(grey, 10.0, 30.0)
    # convert image to binary
    th, binary = cv.threshold(edges, 0, 255, cv.THRESH_BINARY)
    # Create mask for floodfill
    flood = binary.copy()
    h, w = flood.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    # Use floodfill
    flood = binary.copy()
    cv.floodFill(flood, mask, (0,0), 255)
    # Invert flood
    inv_flood = cv.bitwise_not(flood)
    #detect shapes
    _, contours, _ = cv.findContours(inv_flood, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    num_circ = 0
    num_rect = 0
    for cnt in contours:
        approx = cv.approxPolyDP(cnt, 0.01*cv.arcLength(cnt, True), True)
        cv.drawContours(frame_c, [approx], 0, (0), 2)
        x = approx.ravel()[0]
        y = approx.ravel()[1]
        if len(approx) == 4:
            cv.putText(frame_c, "Rectangle", (x, y), font, 1, (0))
            num_rect += 1
        else:
            #cv.putText(frame_c, "Circle", (x, y), font, 1, (0))
            num_circ += 1
    print num_circ
    print num_rect
    aux = True
    while(aux):
        cv.imshow("img_color", inv_flood)
        cv.imshow("img_depth", frame_c)
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
    cv.normalize(frame_d, frame_d, 0, 1, cv.NORM_MINMAX)
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
      #depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback)

      rospy.spin()


  except rospy.ROSInterruptException:
    print "program interrupted before completion"
