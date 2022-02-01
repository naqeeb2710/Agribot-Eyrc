#! /usr/bin/env python3
import sys
import rospy
import moveit_commander

import geometry_msgs.msg
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('arm_manipulation', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_group = moveit_commander.MoveGroupCommander("arm")
hand_group = moveit_commander.MoveGroupCommander("gripper")

arm_group.set_named_target("turn_left")
plan1=arm_group.go()
ur5_pose = geometry_msgs.msg.Pose()
ur5_pose.position.x = 71
ur5_pose.position.y = 29
ur5_pose.position.z = 12
ur5_pose.orientation.x =0
ur5_pose.orientation.y = 0
ur5_pose.orientation.z = 0
ur5_pose.orientation.w = 0
arm_group.set_pose_target(ur5_pose)
plan1 = arm_group.go()

rospy.sleep(5)
moveit_commander.roscpp_shutdown()