#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from gb_visual_detection_3d_msgs.srv import *
from darknet_ros_msgs.msg import BoundingBox

class obj_server:
    def __init__(self):
        self.srv = rospy.ServiceProxy('/GGEZ/dn3d_service',Detect3d)
        self.msg = rospy.Subscriber('/cloud_pcd',sensor_msgs.msg.PointCloud2,self.pointCB)

    def pointCB(self,pcl):
        new_pcl = sensor_msgs.msg.PointCloud2()
        k = raw_input()
        if k == "a":
            print("Sending Service")
            new_pcl = pcl
            new_pcl.header.stamp = pcl.header.stamp
            new_pcl.header.frame_id = "/camera_link"

            xmin = 224
            ymin = 159
            xmax = 587
            ymax = 320

            a = BoundingBox(0.9,xmin,ymin,xmax,ymax,39,'bottle')
            res = self.srv(point_cloud= new_pcl,bounding_box= a)
            
            BBX = self.obj2pose(res.bounding_box_3d)
            print(BBX[0])
            print(BBX[1])

    def obj2pose(self,bbx):
        pose_bbx = Pose()
        xmax = bbx.xmax
        xmin = bbx.xmin
        ymax = bbx.ymax
        ymin = bbx.ymin
        zmax = bbx.zmax
        zmin = bbx.zmin

        x = (xmax + xmin)/2
        y = (ymax + ymin)/2
        z = (zmax + zmin)/2
        l = xmax - xmin
        h = ymax - ymin
        d = zmax - zmin

        pose_bbx.position.x = x
        pose_bbx.position.y = y
        pose_bbx.position.z = z
        pose_bbx.orientation.x = 0.0
        pose_bbx.orientation.y = 0.0
        pose_bbx.orientation.z = 0.0
        pose_bbx.orientation.w = 1.0

        size = (l,h,d)

        return(pose_bbx,size)

if __name__ == "__main__":
    rospy.init_node('objdetect')
    o = obj_server()
    print("Start object detection")
    rospy.spin()