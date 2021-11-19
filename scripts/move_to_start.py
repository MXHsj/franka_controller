#!/usr/bin/env python

import math
import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray

if __name__ == '__main__':
  rospy.init_node('move_to_start')
  rospy.wait_for_message('move_group/status', GoalStatusArray)
  commander = MoveGroupCommander('panda_arm')
  # franka default home pose
  # commander.set_named_target('ready')
  joint_goal = commander.get_current_joint_values()

  # bedside US configuration
  joint_goal[0] = 0.0
  joint_goal[1] = -math.pi/6
  joint_goal[2] = 0.0
  joint_goal[3] = -2*math.pi/3
  joint_goal[4] = 0.0
  joint_goal[5] = math.pi/2
  joint_goal[6] = -math.pi/3

  commander.go(joint_goal, wait=True)
  commander.stop()
