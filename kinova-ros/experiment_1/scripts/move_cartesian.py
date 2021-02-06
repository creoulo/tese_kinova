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
import experiment_1_msgs.srv

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from tf.transformations import *
from std_msgs.msg import *
from trajectory_msgs.msg import *
from moveit_msgs.msg import *
from moveit_commander import *
from geometry_msgs.msg import *
from math import *

#GLOBAL VALUES
GRIPPER_OPEN = 0.7
GRIPPER_CLOSE = 1.1

def argumentParser(argument):
  """ Argument parser """
  parser = argparse.ArgumentParser(description='Drive robot joint to command position')
  #parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                    #help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
  parser.add_argument('x_pick', metavar='x', type=str,
                    help='Insert x coordinate in meters')
  parser.add_argument('y_pick', metavar='y', type=str,
                    help='Insert y coordinate in meters')
  parser.add_argument('z_pick', metavar='z', type=str,
                    help='Insert z coordinate in meters')
  parser.add_argument('vel', metavar='v', type=str,
                    help='Insert velocity between 0 and 2')

  #args_ = parser.parse_args(argument)
  argv = rospy.myargv()
  args_ = parser.parse_args(argv[1:])
  prefix = "j2n6s300"#args_.kinova_robotType
  nbJoints = 6#int(args_.kinova_robotType[3])
  nbfingers = 3#int(args_.kinova_robotType[5])
  x = float(args_.x_pick)
  y = float(args_.y_pick)
  z = float(args_.z_pick)
  v = float(args_.vel)
  target_pose = Pose()
  target_pose.position.x = x
  target_pose.position.y = y
  target_pose.position.z = z
  q = quaternion_from_euler(0, -pi, 0)
  target_pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

  return prefix, nbJoints, nbfingers, target_pose, v

def moveJoint (jointcmds, vel):
  prefix = 'j2n6s300'
  nbJoints = 6
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
    point.accelerations.append(vel)
    point.effort.append(0)
  jointCmd.points.append(copy.deepcopy(point))

  rate = rospy.Rate(100)
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()

def moveFingers (jointcmds):
  prefix = 'j2n6s300'
  nbFingers = 3
  topic_name = '/' + prefix + '/effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  jointCmd = JointTrajectory()
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbFingers):
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

if __name__ == '__main__':
  try:
    rospy.init_node('move_cartesian')
    prefix, nbJoints, nbfingers, target_pose, vel = argumentParser(None)
    req_path = rospy.ServiceProxy('/experiment_1_msgs/CartPath', experiment_1_msgs.srv.CartPath)

    waypoints = []
    waypoints.append(target_pose)
    resp_path = req_path(waypoints)

    for i in range(0, len(resp_path.path.joint_trajectory.points)-1):
        jointcmds = resp_path.path.joint_trajectory.points[i].positions
        moveJoint(jointcmds, vel)


  except rospy.ROSInterruptException:
    print "program interrupted before completion"
