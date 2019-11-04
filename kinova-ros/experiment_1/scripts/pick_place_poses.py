#! /usr/bin/env python
"""Publishes joint trajectory to move robot to given pose"""

import roslaunch
import rospy
import copy
import argparse
import os
import tf
import actionlib

from tf.transformations import *
from std_msgs.msg import *
from trajectory_msgs.msg import *
from moveit_msgs.msg import *
from moveit_commander import *
from kinova_msgs.msg import *
from kinova_msgs.srv import *
from geometry_msgs.msg import *
from math import *

#GLOBAL VALUES
GRIPPER_OPEN = 0.8
GRIPPER_CLOSE = 1.1

def argumentParser(argument):
  """ Argument parser """
  parser = argparse.ArgumentParser(description='Drive robot joint to command position')
  #parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6s300',
                    #help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
  parser.add_argument('x_pick', metavar='x', type=str,
                    help='Insert x coordinate in meters')
  parser.add_argument('y_pick', metavar='y', type=str,
                    help='Insert y coordinate in meters')
  parser.add_argument('z_pick', metavar='z', type=str,
                    help='Insert z coordinate in meters')

  parser.add_argument('x_place', metavar='x', type=str,
                    help='Insert x coordinate in meters')
  parser.add_argument('y_place', metavar='y', type=str,
                    help='Insert y coordinate in meters')
  parser.add_argument('z_place', metavar='z', type=str,
                    help='Insert z coordinate in meters')

  argv = rospy.myargv()
  args_ = parser.parse_args(argv[1:])
  #prefix = args_.kinova_robotType
  #nbJoints = int(args_.kinova_robotType[3])
  #nbfingers = int(args_.kinova_robotType[5])
  x = float(args_.x_place)
  y = float(args_.y_place)
  z = float(args_.z_place)

  x = float(args_.x_pick)
  y = float(args_.y_pick)
  z = float(args_.z_pick)

  return

def moveFingers (jointcmds):
  prefix = 'j2n6s300'
  nbJoints = 3
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

if __name__ == '__main__':
  try:
    rospy.init_node("set_to_pose", anonymous = True)
    #get goal position
    x, y, z, g =  argumentParser(None)

    #initiate moveIt!
    robot = RobotCommander()#provides info on the kinematic model and current joint states
    scene = PlanningSceneInterface()#This provides a remote interface for getting, setting,and updating the robot's internal understanding of the surrounding world

    #arm
    group_arm = "arm"
    move_group_arm = MoveGroupCommander(group_arm) #this interface can be used to plan and execute motions
    #move_group_arm.set_planner_id("ESTkConfigDefault")#planner
    move_group_arm.allow_replanning(True) #Moveit! will try up to five different plans to move the end_effector to the desired pose
    end_effector = move_group_arm.get_end_effector_link()

    #gripper
    group_gripper = "gripper"
    move_group_gripper = MoveGroupCommander(group_gripper) #this interface can be used to plan and execute motions
    #move_group_gripper.set_planner_id("ESTkConfigDefault")#planner
    touch_links = robot.get_link_names(group = group_gripper)

    #display in rviz
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    print end_effector#move_group_arm.get_current_pose(end_effector_link=end_effector)
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    q = quaternion_from_euler(0, -pi, 0)
    p.pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

    if g == "Open" :
        rospy.loginfo("Open grasp")
        #move_group_gripper.set_named_target(g)
        #move_group_gripper.go(wait=True)
        #move_group_gripper.stop()
        moveFingers([GRIPPER_OPEN, GRIPPER_OPEN, GRIPPER_OPEN])

    rospy.loginfo("Going to pose")
    move_group_arm.set_pose_target(p)
    move_group_arm.go(wait=True)
    move_group_arm.clear_pose_targets()
    move_group_arm.stop()

    if g == "Close" :
        rospy.loginfo("Close grasp")
        moveFingers([GRIPPER_CLOSE, GRIPPER_CLOSE, GRIPPER_CLOSE])

        initial = p.pose.position.z
        p.pose.position.z = initial + 0.15
        rospy.loginfo("Going to pose")
        move_group_arm.set_pose_target(p)
        move_group_arm.go(wait=True)
        move_group_arm.clear_pose_targets()
        move_group_arm.stop()

        p.pose.position.x = p.pose.position.x + 0.10
        p.pose.position.y = p.pose.position.y - 0.10
        p.pose.position.z = initial
        rospy.loginfo("Going to pose")
        move_group_arm.set_pose_target(p)
        move_group_arm.go(wait=True)
        move_group_arm.clear_pose_targets()
        move_group_arm.stop()

        moveFingers([GRIPPER_OPEN, GRIPPER_OPEN, GRIPPER_OPEN])



  except rospy.ROSInterruptException:
    print "program interrupted before completion"
