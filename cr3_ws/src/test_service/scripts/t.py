#!/usr/bin/env python

import rospy
from gb_visual_detection_3d_msgs.msg import *

def t(y):
    print(type(y))
    print(len(y.bounding_boxes))


rospy.init_node('objreq_server')
subscriber = rospy.Subscriber('/darknet_ros_3d/bounding_boxes',BoundingBoxes3d,t)

rospy.spin()

