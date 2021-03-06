#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
import argparse
import os
import numpy as np

from std_msgs.msg import *
from geometry_msgs.msg import *
from math import *
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import *


poses_red = []
red_model_name = ['red_cylinder','red_cylinder_clone', 'red_cylinder_clone_0', 'red_cylinder_clone_1', 'red_cylinder_clone_1_clone']
poses_blue = []
game_state = ["","","","","","","","",""]
h = str(0.11) #height of the pieces
win = [(0,1,2),(3,4,5),(6,7,8),(0,3,6),(1,4,7),(1,5,8),(0,4,8),(2,4,6)]

def argumentParser(argument):
  """ Argument parser """
  parser = argparse.ArgumentParser(description='Play tic tac toe')
  #parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                    #help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')


  #args_ = parser.parse_args(argument)
  argv = rospy.myargv()
  args_ = parser.parse_args(argv[1:])
  prefix = "j2n6s300"#args_.kinova_robotType
  nbJoints = 6#int(args_.kinova_robotType[3])
  nbfingers = 3#int(args_.kinova_robotType[5])

  return prefix, nbJoints, nbfingers

def moveJoint (jointcmds,prefix,nbJoints):
  topic_name = '/' + prefix + '/effort_joint_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0)
  jointCmd.points.append(copy.deepcopy(point))

  rate = rospy.Rate(100)
  count = 0
  while (count < 50):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()

def moveFingers (jointcmds,prefix,nbJoints):
  topic_name = '/' + prefix + '/effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_finger_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0)
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()

def getPieces ():
    #gets the initial position of the pieces
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

def getPosBoard (r,c):
    #gets the position of the board
    x = 0
    y = 0
    p_update = 9
    if r == 1:
        if c == 1:
            x = 0.098700
            y = -0.498752
            p_update = 0
        elif c == 2:
            x = -0.005644
            y = -0.498752
            p_update = 1
        elif c == 3:
            x = -0.105557
            y = -0.498752
            p_update = 2
    elif r == 2:
        if c == 1:
            x = 0.098700
            y = -0.406524
            p_update = 3
        elif c == 2:
            x = -0.005644
            y = -0.406524
            p_update = 4
        elif c == 3:
            x = -0.105557
            y = -0.406524
            p_update = 5
    elif r == 3:
        if c == 1:
            x = 0.098700
            y = -0.308745
            p_update = 6
        elif c == 2:
            x = -0.005644
            y = -0.308745
            p_update = 7
        elif c == 3:
            x = -0.105557
            y = -0.308745
            p_update = 8

    return x, y, p_update

def gameStateUpdate(p_update, color):
    game_state[p_update] = color

def endGame():
    #verifies state of the game: set to true when it's tie or anyone wins
    for position in win:
        a, b, c = position
        if game_state[a] == game_state[b] and game_state[b] == game_state[c] and game_state[a] != "":
            if game_state[a] == "red":
                return "WIN RED"
            else:
                return "WIN BLUE"

    res = all(board != "" for board in game_state)
    if res == True:
        return "TIE"

    return "NOTHING"

def moveFromHuman(x, y):
    state_msg = ModelState()
    state_msg.model_name = red_model_name.pop(0)
    state_msg.reference_frame = 'world'
    state_msg.pose.position.x = x
    state_msg.pose.position.y = y
    state_msg.pose.position.z = 0.20
    q = quaternion_from_euler(0, -pi, 0)
    state_msg.pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
  try:
    #--------------------------------------------------------------------------
    rospy.init_node('gameplay')
    prefix, nbJoints, nbfingers = argumentParser(None)
    #allow gazebo to launch
    rospy.sleep(1)

    #Initialization------------------------------------------------------------
    #Define initial poses
    getPieces()

    #Gameplay-----------------------------------------------------------------
    end_game = endGame()
    color = "red" #RED pieces always start

    while end_game == "NOTHING":
        r, c = raw_input("What's the play?(insert row and column)").split()
        #x, y = getPosPiece(color)
        #x = str(x)
        #y = str(y)
        rx, ry, update_board = getPosBoard(int(r), int(c))
        if color == "blue":
            #pick vars
            x, y = poses_blue.pop(0)
            x = str(x)
            y = str(y)
            #place vars
            rx = str(rx)
            ry = str(ry)
            #execute pick and place
            cmd = 'rosrun experiment_1 pickup_place_cartesian.py ' + x + ' ' + y + ' ' + h + ' ' + rx + ' ' + ry + ' ' + h
            os.system(cmd)
        elif color == "red":
            moveFromHuman(rx, ry)

        gameStateUpdate(update_board, color)
        end_game = endGame()
        print end_game
        #change the payer turn
        if color == "red":
            color = "blue"
        else:
            color = "red"


  except rospy.ROSInterruptException:
    print "program interrupted before completion"
