#!/usr/bin/env python

from hashlib import new
import rospy
from sensor_msgs.msg import PointCloud2
from gb_visual_detection_3d_msgs.srv import *
from darknet_ros_msgs.msg import BoundingBox

class l:
    def __init__(self):
        self.srv = rospy.ServiceProxy('/GGEZ/dn3d_service',Detect3d)
        self.msg = rospy.Subscriber('/cloud_pcd',sensor_msgs.msg.PointCloud2,self.pointCB)

    def pointCB(self,pcl):
        new_pcl = sensor_msgs.msg.PointCloud2()
        k = raw_input()
        if k == "a":
            print("in")
            new_pcl = pcl
            new_pcl.header.stamp = pcl.header.stamp
            new_pcl.header.frame_id = "/camera_link"
            print(new_pcl)
            
            a = BoundingBox(0.358,200,100,587,321,63,'laptop')
            res = self.srv(point_cloud= new_pcl,bounding_box= a)
            print(res.bounding_box_3d)


if __name__ == "__main__":
    rospy.init_node('objdetect')
    o = l()
    print("Start object detection")
    rospy.spin()