#! /usr/bin/env python
"""starts moveit_commander"""

import rospy, sys
from moveit_commander import *
from moveit_msgs.msg import *
from experiment_1_msgs.srv import *

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
   (path, q) = move_group_arm.compute_cartesian_path(req.waypoints, 0.005, 0, True)
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


if __name__ == '__main__':
    try:
        rospy.init_node("init_services")
        roscpp_initialize(sys.argv)
        move_group_arm = MoveGroupCommander("arm")
        move_group_gripper = MoveGroupCommander("gripper")

        initServices(move_group_arm, move_group_gripper)

        rospy.spin()
    except rospy.ROSInterruptException:
      print "program ini_group interrupted before completion"
