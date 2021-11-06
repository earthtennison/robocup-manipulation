#!/usr/bin/env python
import moveit_commander
import time
import sys
import rospy

rospy.init_node('node_test_python', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
while True:
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo(current_pose)
    print("- "*20)
    time.sleep(0.5)
