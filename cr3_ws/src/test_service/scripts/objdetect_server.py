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


class GetObjectPose():
    def __init__(self, object_name):

        rospy.loginfo("connecting object detection server")
        # connect to server
        # host = socket.gethostname()
        # connect to acer nitro 5
        host = "192.168.8.2"
        port = 10001
        self.c = CustomSocket(host, port)
        self.c.clientConnect()
        rospy.loginfo("connected object detection server")

        self.object_name = object_name
        self.x_pixel = None
        self.y_pixel = None
        self.intrinsics = None

        self.bridge = CvBridge()
        self.frame = None
        self.is_done = False
        self.object_pose = Pose()

        self.tf_stamp = None
        self.is_trigger = False

        self.time_now = rospy.Time.now()

        # subcriber image delay problem
        #https://stackoverflow.com/questions/26415699/ros-subscriber-not-up-to-date

        rospy.loginfo('Initialize state GetObjectPose')
        depth_info_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info", CameraInfo, self.info_callback)
        self.image_pub = rospy.Publisher(
            "/blob/image_blob", Image, queue_size=1)
        self.pub_tf = rospy.Publisher(
            "/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.yolo_callback, queue_size=1, buff_size=52428800)
        self.depth_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback, queue_size=1, buff_size=52428800)

    def run_once(self):
        while self.intrinsics is None:
            time.sleep(0.1)
        self.c.req(np.random.randint(255, size=(
            self.intrinsics.height, self.intrinsics.width, 3), dtype=np.uint8))

    def reset(self):
        self.x_pixel = None
        self.y_pixel = None
        self.frame = None
        self.is_done = False
        rospy.sleep(0.1)
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.yolo_callback, queue_size=1, buff_size=52428800)
        self.depth_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback, queue_size=1, buff_size=52428800)

    def detect(self):
        self.is_trigger = True
        while not self.is_done:
            rospy.sleep(0.1)
        self.image_sub.unregister()
        self.depth_sub.unregister()
        self.is_trigger = False
        self.is_done = False
        return self.object_pose

    def check_image_size(self, frame):
        # using realsense default
        if frame.shape[0] != self.intrinsics.height and frame.shape[1] != self.intrinsics.width:
            frame = cv2.resize(
                frame, (self.intrinsics.width, self.intrinsics.height))
        return frame

    def info_callback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

    def depth_callback(self, frame):
        """
        +Z           
        |         +y   realsense frame
        |         | +z   o object
        |         |/
        |  +X     o---> +x
        | / 
        0/-----------------> -Y camera_link frame tf

        """
        try:
            if not self.is_trigger:
                if self.tf_stamp is not None:
                    # rospy.loginfo("publishing tf")
                    self.tf_stamp.header.stamp = rospy.Time.now()
                    self.pub_tf.publish(
                        tf2_msgs.msg.TFMessage([self.tf_stamp]))
                return
            elif not ((self.x_pixel is None) or (self.y_pixel is None)):
                depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
                depth_image = self.check_image_size(depth_image)
                # pick one pixel among all the pixels with the closest range:
                pix = (self.x_pixel, self.y_pixel)
                # line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
                if self.intrinsics:
                    depth = depth_image[pix[1], pix[0]]
                    result = [0, 0, 0]
                    result = rs2.rs2_deproject_pixel_to_point(
                        self.intrinsics, [pix[0], pix[1]], depth)
                    x_coord, y_coord, z_coord = result[0] / \
                        1000, result[1]/1000, result[2]/1000
                    # line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
                    if z_coord >= 0.5:

                        # line += '\r'
                        # print(line)
                        rospy.sleep(0.1)

                        self.tf_stamp = TransformStamped()
                        self.tf_stamp.header.frame_id = "/camera_link"
                        self.tf_stamp.header.stamp = rospy.Time.now()
                        self.tf_stamp.child_frame_id = "/object_frame"
                        self.tf_stamp.transform.translation.x = z_coord
                        self.tf_stamp.transform.translation.y = -x_coord
                        self.tf_stamp.transform.translation.z = -y_coord

                        quat = tf.transformations.quaternion_from_euler(
                            float(0), float(0), float(0))

                        self.tf_stamp.transform.rotation.x = quat[0]
                        self.tf_stamp.transform.rotation.y = quat[1]
                        self.tf_stamp.transform.rotation.z = quat[2]
                        self.tf_stamp.transform.rotation.w = quat[3]

                        # set object pose
                        self.object_pose.position.x = z_coord
                        self.object_pose.position.y = -x_coord
                        self.object_pose.position.z = -y_coord
                        self.object_pose.orientation.x = 0
                        self.object_pose.orientation.y = 0
                        self.object_pose.orientation.z = 0
                        self.object_pose.orientation.w = 1

                        self.is_done = True

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
        pass

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
                    self.BBX = (bbox[0], bbox[1],bbox[2], bbox[3])

            # self.frame, x, y, w, h = simple_detect_bbox(self.frame, "blue")
            # self.x_pixel = x
            # self.y_pixel = y

class obj_server:
    def __init__(self):
        self.srv = rospy.ServiceProxy('/GGEZ/dn3d_service',Detect3d)
        self.msg = rospy.Subscriber('/camera/depth_registered/points',PointCloud2,self.pointCB)
        self.test = rospy.Publisher("/anytest",PointCloud2,queue_size= 1)
        self.maker = rospy.Publisher("/markeranytest",Marker,queue_size= 1)
        
        

    def pointCB(self,pcl):
        new_pcl = sensor_msgs.msg.PointCloud2()
        
        raw_input("press enter")

        rospy.loginfo("cv")
        obj = GetObjectPose("Waterbottle")
        obj.run_once()
        self.BBX = obj.BBX
        obj.reset()


        print("Sending Service")
        new_pcl = pcl
        new_pcl.header.stamp = pcl.header.stamp
        new_pcl.header.frame_id = "/object_frame"

        self.test.publish(new_pcl)


        xmin = self.BBX[0]
        ymin = self.BBX[1]
        xmax = self.BBX[2]
        ymax = self.BBX[3]



        # xmin = 480
        # ymin = 300
        # xmax = 580
        # ymax = 368

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