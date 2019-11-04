#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import rospy
import copy
import argparse
import kinova_msgs.msg
import kinova_msgs.srv
import os
import actionlib
import tf2_ros
import tf

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from gazebo_msgs.msg import *
from tf.transformations import *
from std_msgs.msg import *
from trajectory_msgs.msg import *
from moveit_msgs.msg import *
from moveit_commander import *
from geometry_msgs.msg import *
from math import *
from tf2_msgs.msg import TFMessage

poses_red = [Pose(), Pose(), Pose(), Pose(), Pose()]
poses_blue = [Pose(), Pose(), Pose(), Pose(), Pose()]
board = Pose()
game_state = [[][][],[][][],[][][]]

def argumentParser(argument):
  """ Argument parser """
  parser = argparse.ArgumentParser(description='Drive robot joint to command position')
  #parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                    #help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
  parser.add_argument('x', metavar='x', type=str,
                    help='Insert x coordinate in meters')
  parser.add_argument('y', metavar='y', type=str,
                    help='Insert y coordinate in meters')
  parser.add_argument('z', metavar='z', type=str,
                    help='Insert z coordinate in meters')

  #args_ = parser.parse_args(argument)
  argv = rospy.myargv()
  args_ = parser.parse_args(argv[1:])
  prefix = "j2n6s300"#args_.kinova_robotType
  nbJoints = 6#int(args_.kinova_robotType[3])
  nbfingers = 3#int(args_.kinova_robotType[5])
  x = float(args_.x)
  y = float(args_.y)
  z = float(args_.z)
  target_pose = Pose()
  target_pose.position.x = x
  target_pose.position.y = y
  target_pose.position.z = z
  q = quaternion_from_euler(0, -pi, 0)
  target_pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

  return prefix, nbJoints, nbfingers, target_pose

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
    pose = Pose()
    pose.position.x = 0.4
    pose.position.y = -0.04
    pose.position.z = 0
    q = quaternion_from_euler(0, 0, 0)
    pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
    poses_red.append(pose)
    pose = Pose()
    pose.position.x = 0.4
    pose.position.y = -0.14
    pose.position.z = 0
    q = quaternion_from_euler(0, 0, 0)
    pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
    poses_red.append(pose)
    pose = Pose()
    pose.position.x = 0.4
    pose.position.y = -0.24
    pose.position.z = 0
    q = quaternion_from_euler(0, 0, 0)
    pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
    poses_red.append(pose)
    pose = Pose()
    pose.position.x = 0.4
    pose.position.y = -0.34
    pose.position.z = 0
    q = quaternion_from_euler(0, 0, 0)
    pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
    poses_red.append(pose)
    pose = Pose()
    pose.position.x = 0.4
    pose.position.y = -0.44
    pose.position.z = 0
    q = quaternion_from_euler(0, 0, 0)
    pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
    poses_red.append(pose)

    pose = Pose()
    pose.position.x = -0.4
    pose.position.y = -0.04
    pose.position.z = 0
    q = quaternion_from_euler(0, 0, 0)
    pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
    poses_blue.append(pose)
    pose = Pose()
    pose.position.x = -0.4
    pose.position.y = -0.14
    pose.position.z = 0
    q = quaternion_from_euler(0, 0, 0)
    pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
    poses_blue.append(pose)
    pose = Pose()
    pose.position.x = -0.4
    pose.position.y = -0.24
    pose.position.z = 0
    q = quaternion_from_euler(0, 0, 0)
    pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
    poses_blue.append(pose)
    pose = Pose()
    pose.position.x = -0.4
    pose.position.y = -0.34
    pose.position.z = 0
    q = quaternion_from_euler(0, 0, 0)
    pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
    poses_blue.append(pose)
    pose = Pose()
    pose.position.x = -0.4
    pose.position.y = -0.44
    pose.position.z = 0
    q = quaternion_from_euler(0, 0, 0)
    pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
    poses_blue.append(pose)

def getPosPiece (piece):
    pick_pose = Pose()
    if piece == "red":
        pick_pose = poses_red.pop(1)
    elif piece =="blue":
        pick_pose = poses_blue.pop(1)

    q = quaternion_from_euler(0, -pi, 0)
    pick_pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

    return pick_pose

def getBoard ():
    #gets the position of the board
    board.position.x = -0.004067
    board.position.y = -0.230397
    board.position.z = 0.00015
    q = quaternion_from_euler(0, 0, 0)
    board.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

def gameState():
    gameState[1][2] = "X"

if __name__ == '__main__':
  try:
    #------------------------------------------------------------------
    rospy.init_node('pickup_place_cartesian')
    prefix, nbJoints, nbfingers, target_pose = argumentParser(None)
    #allow gazebo to launch
    rospy.sleep(1)

    #Define initial poses
    getPieces()

    #Get board position
    getBoard()

    gameState()
    print game_state


    '''
    #Get the trajectory--------------------------------------
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.sleep(3)
    trans = tfBuffer.lookup_transform("root", "j2n6s300_end_effector", rospy.Time())
    current_pose = Pose()
    current_pose.position = trans.transform.translation
    current_pose.orientation = trans.transform.rotation

    group_arm = "arm"
    move_group_arm = MoveGroupCommander(group_arm)

    waypoints = []
    waypoints.append(current_pose)
    waypoints.append(target_pose)

    (path, fraction) = move_group_arm.compute_cartesian_path(waypoints, 0.01, 0, True)

    print "There will be ", len(path.joint_trajectory.points), "moves"

    for i in range(0, len(path.joint_trajectory.points)-1):
        jointcmds = path.joint_trajectory.points[i].positions
        moveJoint(jointcmds,prefix,nbJoints)
    '''
    #Verify game state

  except rospy.ROSInterruptException:
    print "program interrupted before completion"
