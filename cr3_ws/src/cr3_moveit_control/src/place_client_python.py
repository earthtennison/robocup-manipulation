#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from cr3_moveit_control.srv import *
import random

########################################
#
# Transform a given input pose from one fixed frame to another
import rospy
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

from tf.transformations import quaternion_from_euler
import math
#
########################################

def place_service(corner11, corner12, corner21, corner22, high):
    rospy.wait_for_service('cr3_place')
    try:
        place = rospy.ServiceProxy('cr3_place', cr3_place)
        res = place(corner11, corner12, corner21, corner22, high)
        return res.success_place
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

########################################
#
def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    # pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def to_rad(deg):
    return deg * math.pi / 180.0
#
#########################################

if __name__ == "__main__":
    while not rospy.is_shutdown():
        raw_input("Press enter")
        # x = input("X ")
        # y = input("y ")
        # z = input("z ")
        # -M_PI / 2, -M_PI / 4, +M_PI / 2
        # row = -1*math.pi/2
        # pitch = -1*math.pi/4
        # yaw = math.pi/2

        # geometry_msgs/Point corner11
        # geometry_msgs/Point corner12
        # geometry_msgs/Point corner21
        # geometry_msgs/Point corner22
        # geometry_msgs/Point high
        # ---
        # bool success_place

        point_goal11 = Point()
        point_goal12 = Point()
        point_goal21 = Point()
        point_goal22 = Point()

        point_goal11.x = -0.99
        point_goal12.x = -0.99
        # point_goal21.x = -0.61
        # point_goal22.x = -0.61
        point_goal21.x = -0.35
        point_goal22.x = -0.35

        point_goal11.y = -0.45
        point_goal12.y = 0.42
        point_goal21.y = -0.45
        point_goal22.y = 0.42

        # high = abs(table - base_link)
        high = Point()
        high.z = 0.15

        ########################################
        #
        # Test Case
        # rospy.init_node("transform_test")

        # my_pose = Pose()
        # my_pose.position.x = -1.0
        # my_pose.position.y = 0.0
        # my_pose.position.z = 0.5
        # row = to_rad(0)
        # pitch = to_rad(0)
        # yaw = to_rad(0)

        # quaternion = quaternion_from_euler(row, pitch, yaw)

        # my_pose.orientation.x = quaternion[0]
        # my_pose.orientation.y = quaternion[1]
        # my_pose.orientation.z = quaternion[2]
        # my_pose.orientation.w = quaternion[3]
        
        # # transformed_pose = transform_pose(my_pose, "fixture", "world")
        # transformed_pose = transform_pose(my_pose, "base_link", "real_sense")
        # print(transformed_pose)

        #
        ########################################

        rospy.loginfo(point_goal11, point_goal12, point_goal21, point_goal22, high)
        success = place_service(point_goal11, point_goal12, point_goal21, point_goal22, high)
        print(success)