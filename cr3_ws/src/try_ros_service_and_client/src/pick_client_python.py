#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math
from try_ros_service_and_client.srv import *

def to_rad(deg):
    return deg * math.pi / 180.0

def pick_service(goal_pose):
    rospy.wait_for_service('pick_success')
    try:
        pick = rospy.ServiceProxy('pick_success', kan_pick)
        res = pick(goal_pose)
        return res.success_grasp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        x = input("X ")
        y = input("y ")
        z = input("z ")
        row = to_rad(input("row "))
        pitch = to_rad(input("pitch "))
        yaw = to_rad(input("yaw "))
        q = quaternion_from_euler(row, pitch, yaw)

        pose_goal = Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        rospy.loginfo(pose_goal)
        success = pick_service(pose_goal)
        print(success)