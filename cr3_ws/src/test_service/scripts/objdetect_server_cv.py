#!/usr/bin/env python

import rospy
import time

#msg
from geometry_msgs.msg import Pose

#realsense
from sensor_msgs.msg import PointCloud2,Image, CameraInfo
import pyrealsense2 as rs2
import cv2
from cv_bridge import CvBridge, CvBridgeError

#rviz msg
from visualization_msgs.msg import Marker

#darknet msg
from darknet_ros_msgs.msg import BoundingBox
from gb_visual_detection_3d_msgs.srv import *

# computer vision
import socket
from custom_socket import CustomSocket
import numpy as np

# tf for visualization
import tf2_msgs
import tf
from geometry_msgs.msg import TransformStamped

# transform to from cameralink to base link
import tf2_ros
import tf2_geometry_msgs

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
            output_pose_stamped = tf_buffer.transform(
                pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

class GetObjectPose:
    def __init__(self, object_name):
        # computer vision
        host = "192.168.8.99"
        port = 10001
        self.c = CustomSocket(host, port)
        self.c.clientConnect()
        rospy.loginfo("connected object detection server")

        self.BBX = None
        self.frame = None
        self.is_trigger = False
        self.is_done = False
        self.object_pose = None
        self.size = None
        self.object_name = object_name
        self.bridge = CvBridge()

        self.srv = rospy.ServiceProxy('/GGEZ/dn3d_service',Detect3d)
        self.msg = rospy.Subscriber('/camera/depth_registered/points',PointCloud2,self.pointCB)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.yolo_callback, queue_size=1, buff_size=52428800)
        self.test = rospy.Publisher("/anytest",PointCloud2,queue_size= 1)
        self.maker = rospy.Publisher("/markeranytest",Marker,queue_size= 1)
        self.image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)


    def run_once(self):
        self.c.req(np.random.randint(255, size=(720, 1280, 3), dtype=np.uint8))

    def reset(self):
        self.x_pixel = None
        self.y_pixel = None
        self.frame = None
        self.is_done = False
        rospy.sleep(0.1)
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.yolo_callback, queue_size=1, buff_size=52428800)
    
    def detect(self):
        self.is_trigger = True
        while not self.is_done:
            rospy.sleep(0.1)
        self.image_sub.unregister()
        self.is_trigger = False
        self.is_done = False
        return self.object_pose

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

        return (pose_bbx,size)
    
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
    
    def check_image_size(self, frame):
        # using realsense default
        if frame.shape[0] != 720 and frame.shape[1] != 1280:
            frame = cv2.resize(frame, (1280, 720))
        return frame

    def pointCB(self,pcl):
        if not self.is_trigger:
            return
        elif not ((self.x_pixel is None) or (self.y_pixel is None)):

            new_pcl = sensor_msgs.msg.PointCloud2()
            new_pcl = pcl
            new_pcl.header.stamp = pcl.header.stamp
            new_pcl.header.frame_id = "/object_frame"

            self.test.publish(new_pcl)

            xmin = self.BBX[0]
            ymin = self.BBX[1]
            xmax = self.BBX[2]
            ymax = self.BBX[3]

            a = BoundingBox(0.9,xmin,ymin,xmax,ymax,39,'bottle')
            res = self.srv(point_cloud= new_pcl,bounding_box= a)
            
            (self.object_pose, self.size) = self.obj2pose(res.bounding_box_3d)
            print(self.object_pose)
            print(self.size)
            self.maarker(self.object_pose, self.size)
            self.is_done = True
        
    def yolo_callback(self, data):
        if not self.is_trigger:
            try:
                if self.frame is not None:
                    self.image_pub.publish(
                        self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
            except CvBridgeError as e:
                print(e)
            return
        elif (self.x_pixel is None) and (self.y_pixel is None):
            # change subscribed data to numpy.array and save it as "frame"
            self.frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.frame = self.check_image_size(self.frame)
            # 2d object detection
            # send frame to server and recieve the result
            result = self.c.req(self.frame)
            # rospy.loginfo("result {}".format(result))
            for bbox in result['bbox_list']:
                if bbox[4] != self.object_name:
                    continue
                else:
                    # TODO change to xywh, now receive xyxy
                    self.x_pixel = int(bbox[0] + (bbox[2]-bbox[0])/2)
                    self.y_pixel = int(bbox[1] + (bbox[3]-bbox[1])/2)
                    # visualize purpose
                    self.frame = cv2.circle(
                        self.frame, (bbox[0], bbox[1]), 5, (0, 255, 0), 2)
                    self.frame = cv2.circle(
                        self.frame, (bbox[0], bbox[3]), 5, (0, 255, 0), 2)
                    self.frame = cv2.circle(
                        self.frame, (bbox[2], bbox[1]), 5, (0, 255, 0), 2)
                    self.frame = cv2.circle(
                        self.frame, (bbox[2], bbox[3]), 5, (0, 255, 0), 2)
                    self.frame = cv2.circle(
                        self.frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)

                    self.frame = cv2.rectangle(
                        self.frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 2)
                    
                    bbox[0] = max(min(bbox[0], 719), 0)
                    bbox[1] = max(min(bbox[0], 1279), 0)
                    bbox[2] = max(min(bbox[0], 719), 0)
                    bbox[3] = max(min(bbox[0], 1279), 0)
                    self.BBX = (bbox[0], bbox[1], bbox[2], bbox[3])
        

if __name__ == "__main__":
    rospy.init_node('objdetect')
    detector = GetObjectPose("Waterbottle")
    detector.run_once()
    while not rospy.is_shutdown():
        command = raw_input("Press Enter: ")
        if command == "q":
            break

        print("running 3d detection")
        detector.reset()
        object_pose = detector.detect()

        ##################
        # transform
        print(object_pose)
        # print("before {}".format(object_pose))
        # object_pose = transform_pose(object_pose, "camera_link", "base_link")