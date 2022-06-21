#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math

# pick service
from cr3_moveit_control.srv import *

# detect 3d esrvice
from gb_visual_detection_3d_msgs.srv import *
from darknet_ros_msgs.msg import BoundingBox

def to_rad(deg):
    return deg * math.pi / 180.0

def pick_service(goal_pose):
    rospy.wait_for_service('cr3_pick')
    try:
        pick = rospy.ServiceProxy('cr3_pick', cr3_pick)
        res = pick(goal_pose)
        return res.success_grasp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

class Detect3dClient:
    def __init__(self):
        rospy.wait_for_service('/GGEZ/dn3d_service')
        self.srv = rospy.ServiceProxy('/GGEZ/dn3d_service',Detect3d)
        self.msg = rospy.Subscriber('/cloud_pcd',sensor_msgs.msg.PointCloud2,self.pointCB)
        self.pcl = sensor_msgs.msg.PointCloud2()

    def pointCB(self,pcl):
        self.pcl = pcl
        self.pcl.header.stamp = pcl.header.stamp
        self.pcl.header.frame_id = "/camera_link"

    def detect3d(self):

        # todo receive 2d detect
        xmin = 224
        ymin = 159
        xmax = 587
        ymax = 320

        bbox2d = BoundingBox(0.9,xmin,ymin,xmax,ymax,39,'bottle')
        res = self.srv(point_cloud= self.pcl,bounding_box= bbox2d)
        bbox3d = self.obj2pose(res.bounding_box_3d)
        print(bbox3d[0])
        print(bbox3d[1])
        return bbox3d

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
    
    rospy.init_node('manipulation_pipeline')
    detect3dclient = Detect3dClient()

    # detect 2d TODO

    #################
    # detect 3d
    pose_goal, size = detect3dclient.detect3d()
    #################
    # pregrasp
    success = pick_service(pose_goal)
