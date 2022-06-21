#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from cr3_moveit_control.srv import *
import random

def place_service(corner11, corner12, corner21, corner22, high):
    rospy.wait_for_service('cr3_place')
    try:
        place = rospy.ServiceProxy('cr3_place', cr3_place)
        res = place(corner11, corner12, corner21, corner22, high)
        return res.success_place
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

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

        point_goal11.x = -1.4
        point_goal12.x = -1.4
        point_goal21.x = -0.4
        point_goal22.x = -0.4

        point_goal11.y = -1.0
        point_goal12.y = 1.0
        point_goal21.y = -1.0
        point_goal22.y = 1.0

        # high = abs(table - base_link)
        high = Point()
        high.z = 0.01

        rospy.loginfo(point_goal11, point_goal12, point_goal21, point_goal22, high)
        success = place_service(point_goal11, point_goal12, point_goal21, point_goal22, high)
        print(success)