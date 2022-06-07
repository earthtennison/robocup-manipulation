#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
from test_service.srv import GetPhoto

rospy.init_node('res_service')
 
rospy.wait_for_service('getph')

def callback(pc):
    
    return
 
srv=rospy.ServiceProxy('getph',GetPhoto)
subpc = rospy.Subscriber('/camera/depth_registered/points',PointCloud,callback)


