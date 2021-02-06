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
from experiment_1_msgs.srv import *
from geometry_msgs.msg import *
from math import *

#GLOBAL VALUES
GRIPPER_OPEN = 0.7 #min 0.0
GRIPPER_CLOSE = 1.3 #max 2.0

def argumentParser(argument):
  """ Argument parser """
  parser = argparse.ArgumentParser(description='Drive robot joint to command cartesian position')
  #parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
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

  #args_ = parser.parse_args(argument)
  argv = rospy.myargv()
  args_ = parser.parse_args(argv[1:])
  prefix = "j2n6s300"#args_.kinova_robotType
  nbJoints = 6#int(args_.kinova_robotType[3])
  nbfingers = 3#int(args_.kinova_robotType[5])
  x = float(args_.x_pick)
  y = float(args_.y_pick)
  z = float(args_.z_pick)
  pick_pose = Pose()
  pick_pose.position.x = x
  pick_pose.position.y = y
  pick_pose.position.z = z
  q = quaternion_from_euler(0, -pi, 0)
  pick_pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

  x = float(args_.x_place)
  y = float(args_.y_place)
  z = float(args_.z_place)
  place_pose = Pose()
  place_pose.position.x = x
  place_pose.position.y = y
  place_pose.position.z = z
  q = quaternion_from_euler(0, -pi, 0)
  place_pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

  return prefix, nbJoints, nbfingers, pick_pose, place_pose

def initServices(move_group_arm, move_group_gripper):
    #Instanciate all the services
    rospy.Service(name="experiment_1_msgs/CartPath", service_class=CartPath,
                                                    handler=cart_path_handler)
    rospy.Service(name="experiment_1_msgs/NamedTargetArm", service_class=NamedTargetArm,
                                                    handler=named_target_arm_handler)
    rospy.Service(name="experiment_1_msgs/NamedTargetGripper", service_class=NamedTargetGripper,
                                                    handler=named_target_gripper_handler)

#define all the handlers
def cart_path_handler(req):
   (path, q) = move_group_arm.compute_cartesian_path(req.waypoints, 0.01, 0, True)
   return CartPathResponse(path, q)

def named_target_arm_handler(req):
    move_group_arm.set_start_state_to_current_state()
    move_group_arm.set_named_target(req.target)
    move_group_arm.go(wait=True)
    move_group_arm.stop()
    success = True
    return NamedTargetArmResponse(success)

def named_target_gripper_handler(req):
    move_group_gripper.set_start_state_to_current_state()
    move_group_gripper.set_named_target(req.target)
    move_group_gripper.go(wait=True)
    move_group_gripper.stop()
    success = True
    return NamedTargetGripperResponse(success)

def moveJoint (jointcmds):
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
    point.accelerations.append(0)
    point.effort.append(0)
  jointCmd.points.append(copy.deepcopy(point))

  rate = rospy.Rate(100)
  count = 0
  while (count < 50):
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
    rospy.init_node('pick_up_and_place')
    #initiate moveIt!
    robot = RobotCommander()#provides info on the kinematic model and current joint states
    scene = PlanningSceneInterface()#This provides a remote interface for getting, setting,and updating the robot's internal understanding of the surrounding world

    rospy.sleep(2)
    #add a "wall" as a collision object to limit the space that the robot can move
    upper_bond_pose = PoseStamped()
    upper_bond_pose.header.frame_id = robot.get_planning_frame()
    upper_bond_pose.pose.position.x = 0
    upper_bond_pose.pose.position.y = 0
    upper_bond_pose.pose.position.z = 0.8
    scene.add_box('upper_bond', upper_bond_pose, size = (2,2,0.001)) #add_box (self, name, pose, size=(1, 1, 1))
    back_bond_pose = PoseStamped()
    back_bond_pose.header.frame_id = robot.get_planning_frame()
    back_bond_pose.pose.position.x = 0
    back_bond_pose.pose.position.y = 0.16
    back_bond_pose.pose.position.z = 0.5
    scene.add_box('back_bond',back_bond_pose, size = (2,0.001,1)) #add_box (self, name, pose, size=(1, 1, 1))
    rospy.sleep(2)

    #arm
    group_arm = "arm"
    move_group_arm = MoveGroupCommander(group_arm) #this interface can be used to plan and execute motions
    move_group_arm.set_planner_id("RRTConnectkConfigDefault")#planner
    move_group_arm.allow_replanning(True) #Moveit! will try up to five different plans to move the end_effector to the desired pose
    #move_group_arm.set_max_velocity_scaling_factor(1)#allowed values between 0 and 1

    #gripper
    group_gripper = "gripper"
    move_group_gripper = MoveGroupCommander(group_gripper)
    #move_group_gripper.set_max_velocity_scaling_factor(1)#allowed values between 0 and 1

    initServices(move_group_arm, move_group_gripper)

    prefix, nbJoints, nbfingers, pick_pose, place_pose = argumentParser(None)
    req_path = rospy.ServiceProxy('/experiment_1_msgs/CartPath', experiment_1_msgs.srv.CartPath )
    go_home = rospy.ServiceProxy('/experiment_1_msgs/NamedTargetArm', experiment_1_msgs.srv.NamedTargetArm)

    #PICK-----------------------------------------------------------------------
    #1 - open gripper
    #2 - move to pick over pose with z = z + 0.2
    #3 - downward movement to pick position
    #4 - close gripper
    moveFingers([GRIPPER_OPEN, GRIPPER_OPEN, GRIPPER_OPEN])

    pick_over_pose = copy.deepcopy(pick_pose)
    pick_over_pose.position.z = 3 * pick_pose.position.z
    rospy.loginfo("Going to pose over piece")
    move_group_arm.set_pose_target(pick_over_pose)
    move_group_arm.go(wait=True)
    move_group_arm.clear_pose_targets()
    move_group_arm.stop()

    waypoints = []
    waypoints.append(pick_pose)
    resp_path = req_path(waypoints)
    while resp_path.quality < 0.8:
        resp_path = req_path(waypoints)

    for i in range(0, len(resp_path.path.joint_trajectory.points)-1):
        jointcmds = resp_path.path.joint_trajectory.points[i].positions
        moveJoint(jointcmds)

    rospy.loginfo("Grabbing piece")
    moveFingers([GRIPPER_CLOSE, GRIPPER_CLOSE, GRIPPER_CLOSE])

    #PLACE----------------------------------------------------------------------
    #1 - upward movement in pick pose
    #2 - move to place pose with z = z + 0.2
    #3 - downward movement in place position
    #4 - open gripper
    rospy.loginfo("Going to pose over piece")
    waypoints = []
    #pick_over_pose.position.z = 2 * pick_pose.position.z
    waypoints.append(pick_over_pose)
    resp_path = req_path(waypoints)
    while resp_path.quality < 0.8:
        resp_path = req_path(waypoints)

    for i in range(0, len(resp_path.path.joint_trajectory.points)-1):
        jointcmds = resp_path.path.joint_trajectory.points[i].positions
        moveJoint(jointcmds)

    place_over_pose = copy.deepcopy(place_pose)
    place_over_pose.position.z = 3 * place_pose.position.z
    rospy.loginfo("Going to place the piece on board")
    move_group_arm.set_pose_target(place_over_pose)
    move_group_arm.go(wait=True)
    move_group_arm.clear_pose_targets()
    move_group_arm.stop()

    waypoints = []
    waypoints.append(place_pose)
    resp_path = req_path(waypoints)
    while resp_path.quality < 0.8:
        resp_path = req_path(waypoints)

    for i in range(0, len(resp_path.path.joint_trajectory.points)-1):
        jointcmds = resp_path.path.joint_trajectory.points[i].positions
        moveJoint(jointcmds)

    moveFingers([GRIPPER_OPEN, GRIPPER_OPEN, GRIPPER_OPEN])

    #HOME-----------------------------------------------------------------------
    #1 - upward movement to place pose with z = 0.02
    #2 - arm to home
    waypoints = []
    waypoints.append(place_over_pose)
    resp_path = req_path(waypoints)
    while resp_path.quality < 0.8:
        resp_path = req_path(waypoints)

    for i in range(0, len(resp_path.path.joint_trajectory.points)-1):
        jointcmds = resp_path.path.joint_trajectory.points[i].positions
        moveJoint(jointcmds)

    go_home("Home")#this movement is not computed with compute_cartesian_path

  except rospy.ROSInterruptException:
    print "program interrupted before completion"
