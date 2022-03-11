#!/usr/bin/env python

import sys 
import rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node('Move', anonymous=True)


robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("xarm6")
arm_group.set_named_target("hold-up")
plan1 = arm_group.go(wait = True)

hand_group = moveit_commander.MoveGroupCommander("xarm_gripper")


hand_group.set_named_target("open")
plan2 = hand_group.go(wait = True)


pose_target = geometry_msgs.msg.Pose()
pose_target.position.x = -0.186303
pose_target.position.y = 0.411113
pose_target.position.z = 1.045
pose_target.orientation.x = -0.000004
pose_target.orientation.y = 0.0000004
pose_target.orientation.z = 0.001828
pose_target.orientation.w = 0.001828

arm_group.set_pose_target(pose_target)
plan1 = arm_group.go()


rospy.sleep(5)
moveit_commander.roscpp_shutdown()