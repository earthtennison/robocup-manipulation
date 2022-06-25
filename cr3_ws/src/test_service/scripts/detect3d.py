#!/usr/bin/env python
from re import X
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from gb_visual_detection_3d_msgs.srv import *
from visualization_msgs.msg import Marker
from darknet_ros_msgs.msg import BoundingBox
class obj_server:
    def __init__(self):
        self.srv = rospy.ServiceProxy('/GGEZ/dn3d_service',Detect3d)
        self.msg = rospy.Subscriber('/camera/depth_registered/points',PointCloud2,self.pointCB)
        self.test = rospy.Publisher("/anytest",PointCloud2,queue_size= 1)
        self.maker = rospy.Publisher("/markeranytest",Marker,queue_size= 1)
    def pointCB(self,pcl):
        new_pcl = sensor_msgs.msg.PointCloud2()

        raw_input()
        print("Sending Service")
        new_pcl = pcl
        new_pcl.header.stamp = pcl.header.stamp
        new_pcl.header.frame_id = "/object_frame"

        self.test.publish(new_pcl)



        xmin = 286
        ymin = 0
        xmax = 312
        ymax = 130

        a = BoundingBox(0.9,xmin,ymin,xmax,ymax,39,'bottle')
        res = self.srv(point_cloud= new_pcl,bounding_box= a)
        # while(True):
        #     a = BoundingBox(0.9,xmin,ymin,xmax,ymax,39,'laptop')
        #     res = self.srv(point_cloud= new_pcl,bounding_box= a)
        #     if res.success == False:
        #         xmax +=1 
        #     else:
        #         break


        BBX = self.obj2pose(res.bounding_box_3d)
        print(BBX[0])
        print(BBX[1])
        self.maarker(BBX[0],BBX[1])


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
    
    def maarker(self,pose_bbx,size):
        marker = Marker()
        marker.header.frame_id = "object_frame"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "obj_server"
        marker.id = 1
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.position.x = pose_bbx.position.x
        marker.pose.position.y = pose_bbx.position.y
        marker.pose.position.z = pose_bbx.position.z
        marker.pose.orientation.x = pose_bbx.orientation.x
        marker.pose.orientation.y = pose_bbx.orientation.y
        marker.pose.orientation.z = pose_bbx.orientation.z
        marker.pose.orientation.w = pose_bbx.orientation.w
        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]
        marker.color.a = 0.4
        marker.color.r = 0.0
        marker.color.g = 255.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(60.0)
        self.maker.publish(marker)
        
        
if __name__ == "__main__":
    rospy.init_node('objdetect')
    o = obj_server()
    print("Start object detection")
    rospy.spin()