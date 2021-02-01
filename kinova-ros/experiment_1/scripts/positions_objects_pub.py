#!/usr/bin/env python
import rospy
import experiment_1_msgs.srv
import experiment_1_msgs.msg
import tf

from experiment_1_msgs.srv import *
from experiment_1_msgs.msg import *
global coordinates_board

def get_board_coord():
    objects = ['object_6']#['object_20', 'object_21', 'object_22', 'object_23', 'object_24', 'object_25', 'object_26', 'object_27']
    coords = InfoBoard()
    listener = tf.TransformListener()
    for object in objects:
        listener.waitForTransform("/world", object, rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('world', object, rospy.Time())
        #coords.coord.append(Coordinates(x = trans[0], y = trans[1]))
        x = trans[0]
        y = trans[1]
    print "Middle square at: ", x, y
    w = 0.1
    square_matrix = [Coordinates(x = x+w, y = y-w),Coordinates(x = x, y = y-w),Coordinates(x = x-w, y = y-w),
                     Coordinates(x = x+w, y = y),Coordinates(x = x, y = y),Coordinates(x = x-w, y = y),
                     Coordinates(x = x+w, y = y+w),Coordinates(x = x, y = y+w),Coordinates(x = x-w, y = y+w)]

    coords.coord = square_matrix
    print "Coordinates:", coords
    return coords

def callback_board(msg):
    global coordinates_board
    coordinates_board = msg.coord

if __name__ == '__main__':
    try:
        coordinates_board = InfoBoard()
        rospy.init_node('positions_info')
        pub1 = rospy.Publisher('board_position', InfoBoard, queue_size=1)#queue_size equal to the rate as the message is going
        rate = rospy.Rate(1)#Hz
        info_board = get_board_coord()

        while not rospy.is_shutdown():
            pub1.publish(info_board)

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
