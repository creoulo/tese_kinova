#!/usr/bin/env python
import experiment_1_msgs.srv
import rospy

from experiment_1_msgs.srv import *

def initServices():
    #Instanciate all the services
    rospy.Service(name="experiment_1_msgs/PosBoard", service_class=PosBoard,
                                                    handler=pos_board_handler)
    rospy.Service(name="experiment_1_msgs/PosPiece", service_class=PosPiece,
                                                    handler=pos_piece_handler)

#define all the handlers
def pos_board_handler(req):
   #gets the position of the board given the row and collumn relative to the world
   x = 0
   y = 0
   pos_update = 9
   if req.r == 1:
       if req.c == 1:
           x = 0.098700
           y = -0.498752
           pos_update = 0
       elif req.c == 2:
           x = -0.005644
           y = -0.498752
           pos_update = 1
       elif req.c == 3:
           x = -0.105557
           y = -0.498752
           pos_update = 2
   elif req.r == 2:
       if req.c == 1:
           x = 0.098700
           y = -0.406524
           pos_update = 3
       elif req.c == 2:
           x = -0.005644
           y = -0.406524
           pos_update = 4
       elif req.c == 3:
           x = -0.105557
           y = -0.406524
           pos_update = 5
   elif req.r == 3:
       if req.c == 1:
           x = 0.098700
           y = -0.308745
           pos_update = 6
       elif req.c == 2:
           x = -0.005644
           y = -0.308745
           pos_update = 7
       elif req.c == 3:
           x = -0.105557
           y = -0.308745
           pos_update = 8
   return PosBoardResponse(x, y, pos_update)

def pos_piece_handler():
    #gets the initial position of the pieces relative to the world and stores them in vectors
    poses_red = []
    poses_blue = []
    #Red
    pose = (-0.302826, -0.139972)
    poses_red.append(pose)
    pose = (-0.299920, -0.239980)
    poses_red.append(pose)
    pose = (-0.300678, -0.339974)
    poses_red.append(pose)
    pose = (-0.300676, -0.442596)
    poses_red.append(pose)
    pose = (-0.301295, -0.540520)
    poses_red.append(pose)

    #Blue
    pose = (-0.4, -0.04)
    poses_blue.append(pose)
    pose = (-0.4, -0.14)
    poses_blue.append(pose)
    pose = (-0.4, -0.24)
    poses_blue.append(pose)
    pose = (-0.4, -0.34)
    poses_blue.append(pose)
    pose = (-0.4, -0.44)
    poses_blue.append(pose)

    return PosPieceResponde(poses_red, poses_blue)

if __name__ == "__main__":
    rospy.init_node('positions_server')
    getPosBoard = rospy.ServiceProxy('/experiment_1_msgs/pos_board', PosBoard, pos_board_handler)
    getPieces = rospy.ServiceProxy('/experiment_1_msgs/pos_piece', PosPiece, pos_piece_handler)

    rospy.spin()
