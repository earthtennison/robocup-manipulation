#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from gb_visual_detection_3d_msgs.srv import *
from darknet_ros_msgs.msg import BoundingBox

class l:
    def __init__(self):
        self.srv = rospy.ServiceProxy('/add_two_ints_server/dn3d_service',Detect3d)
        self.msg = rospy.Subscriber('/cloud_pcd',sensor_msgs.msg.PointCloud2,self.pointCB)

    def pointCB(self,pcl):
        k = raw_input()
        if k == "a":
            print("in")
            a = BoundingBox(1,1,1,2,2,123,'box')
            res = self.srv(point_cloud= self.msg,bounding_box= a)
            print(res.bounding_box_3d)


if __name__ == "__main__":
    rospy.init_node('objdetect')
    o = l()
    print("Start object detection")
    rospy.spin()