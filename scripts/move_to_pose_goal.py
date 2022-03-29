#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

if __name__ == '__main__':
  rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
  robot = moveit_commander.RobotCommander()
  group_name = "panda_arm"
  move_group = moveit_commander.MoveGroupCommander(group_name)

  planning_frame = move_group.get_planning_frame()
  print("============ Planning frame: %s" % planning_frame)
  eef_link = move_group.get_end_effector_link()
  print("============ End effector link: %s" % eef_link)
  group_names = robot.get_group_names()
  print("============ Available Planning Groups:", robot.get_group_names())
  current_pose = move_group.get_current_pose().pose
  print(current_pose)

  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.position.x = 0.4
  pose_goal.position.y = 0.0
  pose_goal.position.z = 0.4
  pose_goal.orientation.w = 1.0
  # pose_goal.orientation.x = -0.7071
  # pose_goal.orientation.y = 0.7071
  # pose_goal.orientation.z = 0.00
  # pose_goal.orientation.w = 0.00

  move_group.set_pose_target(pose_goal)

  plan = move_group.go(wait=True)
  move_group.stop()
  move_group.clear_pose_targets()
