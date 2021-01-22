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
        listener.waitForTransform("/world", object, rospy.Time(), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform('world', object, rospy.Time())
        coords.coord.append(Coordinates(x = trans[0], y = trans[1]))
    return coords

def callback_board(msg):
    global coordinates_board
    coordinates_board = msg.coord

if __name__ == '__main__':
    try:
        coordinates_board = InfoBoard()
        rospy.init_node('positions_info')
        pub1 = rospy.Publisher('board_position', InfoBoard, queue_size=100)#queue_size equal to the rate as the message is going
        rate = rospy.Rate(100)#Hz
        info_board = get_board_coord()

        while not rospy.is_shutdown():
            pub1.publish(info_board)
            print "pub:", info_board
            rospy.Subscriber("board_position", InfoBoard, callback_board)
            print "\nsub1: ", coordinates_board

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
