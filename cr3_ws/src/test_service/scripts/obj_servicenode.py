#!/usr/bin/env python

import rospy
from test_service.srv import GetPhoto

def cb(pcl):
    return rospy.loginfo('dee')
rospy.init_node('obj_respond')
service=rospy.Service('getph',GetPhoto,cb)
rospy.spin()