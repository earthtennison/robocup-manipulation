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


class GetObjectPose():
    def __init__(self, object_name):

        rospy.loginfo("connecting object detection server")
        # connect to server
        # host = socket.gethostname()
        # connect to acer nitro 5
        host = "192.168.8.99"
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
        self.corner11_pose = Pose()
        self.corner12_pose = Pose()
        self.corner21_pose = Pose()
        self.corner22_pose = Pose()

        self.tf_stamp = None
        self.tf_stamp11 = None
        self.tf_stamp12 = None
        self.tf_stamp21 = None
        self.tf_stamp22 = None

        self.BBXX = None

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
        return (self.object_pose, self.corner11_pose, self.corner12_pose, self.corner21_pose, self.corner22_pose)

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

                    self.tf_stamp11.header.stamp = rospy.Time.now()
                    self.tf_stamp12.header.stamp = rospy.Time.now()
                    self.tf_stamp21.header.stamp = rospy.Time.now()
                    self.tf_stamp22.header.stamp = rospy.Time.now()
                    
                    self.pub_tf.publish(
                        tf2_msgs.msg.TFMessage([self.tf_stamp]))
                    self.pub_tf.publish(
                        tf2_msgs.msg.TFMessage([self.tf_stamp11]))
                    self.pub_tf.publish(
                        tf2_msgs.msg.TFMessage([self.tf_stamp12]))
                    self.pub_tf.publish(
                        tf2_msgs.msg.TFMessage([self.tf_stamp21]))
                    self.pub_tf.publish(
                        tf2_msgs.msg.TFMessage([self.tf_stamp22]))
                return

            elif not ((self.x_pixel is None) or (self.y_pixel is None)):
                depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
                depth_image = self.check_image_size(depth_image)
                # pick one pixel among all the pixels with the closest range:
                pix = (self.x_pixel, self.y_pixel)
                corne11 = (self.BBXX[0],self.BBXX[1])
                corne12 = (self.BBXX[0],self.BBXX[3])
                corne21 = (self.BBXX[2],self.BBXX[1])
                corne22 = (self.BBXX[2],self.BBXX[3])
                # line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
                if self.intrinsics:
                    depth = depth_image[pix[1], pix[0]]
                    # depth_corne11 = depth_image[corne11[1], corne11[0]]
                    # depth_corne12 = depth_image[corne12[1], corne12[0]]
                    # depth_corne21 = depth_image[corne21[1], corne21[0]]
                    # depth_corne22 = depth_image[corne22[1], corne22[0]]

                    result = [0, 0, 0]

                    result_corner11 = [0, 0, 0]
                    result_corner12 = [0, 0, 0]
                    result_corner21 = [0, 0, 0]
                    result_corner22 = [0, 0, 0]

                    result = rs2.rs2_deproject_pixel_to_point(
                        self.intrinsics, [pix[0], pix[1]], depth)

                    result_corner11 = rs2.rs2_deproject_pixel_to_point(
                        self.intrinsics, [corne11[0], corne11[1]], depth)
                    result_corner12 = rs2.rs2_deproject_pixel_to_point(
                        self.intrinsics, [corne12[0], corne12[1]], depth)
                    result_corner21 = rs2.rs2_deproject_pixel_to_point(
                        self.intrinsics, [corne21[0], corne21[1]], depth)
                    result_corner22 = rs2.rs2_deproject_pixel_to_point(
                        self.intrinsics, [corne22[0], corne22[1]], depth)


                    x_corne11, y_corne11, z_corne11 = result_corner11[0] /1000, result_corner11[1]/1000, result_corner11[2]/1000

                    x_corne12, y_corne12, z_corne12 = result_corner12[0] /1000, result_corner12[1]/1000, result_corner12[2]/1000

                    x_corne21, y_corne21, z_corne21 = result_corner21[0] /1000, result_corner21[1]/1000, result_corner21[2]/1000

                    x_corne22, y_corne22, z_corne22 = result_corner22[0] /1000, result_corner22[1]/1000, result_corner22[2]/1000


                    x_coord, y_coord, z_coord = result[0] / \
                        1000, result[1]/1000, result[2]/1000
                    # line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
                    if z_coord >= 0.5:

                        # line += '\r'
                        # print(line)
                        rospy.sleep(0.1)

                        quat = tf.transformations.quaternion_from_euler(
                            float(0), float(0), float(0))

                        self.tf_stamp = TransformStamped()
                        self.tf_stamp.header.frame_id = "/camera_link"
                        self.tf_stamp.header.stamp = rospy.Time.now()
                        self.tf_stamp.child_frame_id = "/object_frame"
                        self.tf_stamp.transform.translation.x = z_coord
                        self.tf_stamp.transform.translation.y = -x_coord
                        self.tf_stamp.transform.translation.z = -y_coord

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

                        #////////////////corner11///////////////////

                        self.tf_stamp11 = TransformStamped()
                        self.tf_stamp11.header.frame_id = "/camera_link"
                        self.tf_stamp11.header.stamp = rospy.Time.now()
                        self.tf_stamp11.child_frame_id = "/corner11_frame"
                        self.tf_stamp11.transform.translation.x = z_corne11
                        self.tf_stamp11.transform.translation.y = -x_corne11
                        self.tf_stamp11.transform.translation.z = -y_corne11

                        self.tf_stamp11.transform.rotation.x = quat[0]
                        self.tf_stamp11.transform.rotation.y = quat[1]
                        self.tf_stamp11.transform.rotation.z = quat[2]
                        self.tf_stamp11.transform.rotation.w = quat[3]

                        # set object pose
                        self.corner11_pose.position.x = z_corne11
                        self.corner11_pose.position.y = -x_corne11
                        self.corner11_pose.position.z = -y_corne11
                        self.corner11_pose.orientation.x = 0
                        self.corner11_pose.orientation.y = 0
                        self.corner11_pose.orientation.z = 0
                        self.corner11_pose.orientation.w = 1

                        #////////////////corner12///////////////////

                        self.tf_stamp12 = TransformStamped()
                        self.tf_stamp12.header.frame_id = "/camera_link"
                        self.tf_stamp12.header.stamp = rospy.Time.now()
                        self.tf_stamp12.child_frame_id = "/corner12_frame"
                        self.tf_stamp12.transform.translation.x = z_corne12
                        self.tf_stamp12.transform.translation.y = -x_corne12
                        self.tf_stamp12.transform.translation.z = -y_corne12

                        self.tf_stamp12.transform.rotation.x = quat[0]
                        self.tf_stamp12.transform.rotation.y = quat[1]
                        self.tf_stamp12.transform.rotation.z = quat[2]
                        self.tf_stamp12.transform.rotation.w = quat[3]

                        # set object pose
                        self.corner12_pose.position.x = z_corne12
                        self.corner12_pose.position.y = -x_corne12
                        self.corner12_pose.position.z = -y_corne12
                        self.corner12_pose.orientation.x = 0
                        self.corner12_pose.orientation.y = 0
                        self.corner12_pose.orientation.z = 0
                        self.corner12_pose.orientation.w = 1

                        #////////////////corner21///////////////////

                        self.tf_stamp21 = TransformStamped()
                        self.tf_stamp21.header.frame_id = "/camera_link"
                        self.tf_stamp21.header.stamp = rospy.Time.now()
                        self.tf_stamp21.child_frame_id = "/corner21_frame"
                        self.tf_stamp21.transform.translation.x = z_corne21
                        self.tf_stamp21.transform.translation.y = -x_corne21
                        self.tf_stamp21.transform.translation.z = -y_corne21

                        self.tf_stamp21.transform.rotation.x = quat[0]
                        self.tf_stamp21.transform.rotation.y = quat[1]
                        self.tf_stamp21.transform.rotation.z = quat[2]
                        self.tf_stamp21.transform.rotation.w = quat[3]

                        # set object pose
                        self.corner21_pose.position.x = z_corne21
                        self.corner21_pose.position.y = -x_corne21
                        self.corner21_pose.position.z = -y_corne21
                        self.corner21_pose.orientation.x = 0
                        self.corner21_pose.orientation.y = 0
                        self.corner21_pose.orientation.z = 0
                        self.corner21_pose.orientation.w = 1

                        #////////////////corner22///////////////////

                        self.tf_stamp22 = TransformStamped()
                        self.tf_stamp22.header.frame_id = "/camera_link"
                        self.tf_stamp22.header.stamp = rospy.Time.now()
                        self.tf_stamp22.child_frame_id = "/corner22_frame"
                        self.tf_stamp22.transform.translation.x = z_corne22
                        self.tf_stamp22.transform.translation.y = -x_corne22
                        self.tf_stamp22.transform.translation.z = -y_corne22

                        self.tf_stamp22.transform.rotation.x = quat[0]
                        self.tf_stamp22.transform.rotation.y = quat[1]
                        self.tf_stamp22.transform.rotation.z = quat[2]
                        self.tf_stamp22.transform.rotation.w = quat[3]

                        # set object pose
                        self.corner22_pose.position.x = z_corne22
                        self.corner22_pose.position.y = -x_corne22
                        self.corner22_pose.position.z = -y_corne22
                        self.corner22_pose.orientation.x = 0
                        self.corner22_pose.orientation.y = 0
                        self.corner22_pose.orientation.z = 0
                        self.corner22_pose.orientation.w = 1



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
                    
                    bbox[0] = max(min(bbox[0], 719), 0)
                    bbox[1] = max(min(bbox[1], 1279), 0)
                    bbox[2] = max(min(bbox[2], 719), 0)
                    bbox[3] = max(min(bbox[3], 1279), 0)
                    self.BBXX = (bbox[0], bbox[1], bbox[2], bbox[3])
                    print(self.BBXX)

            # self.frame, x, y, w, h = simple_detect_bbox(self.frame, "blue")
            # self.x_pixel = x
            # self.y_pixel = y      
        

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
        (object_pose, corner11_pose, corner12_pose, corner21_pose, corner22_pose) = detector.detect()

        ##################
        # transform
        # print(object_pose)
        rospy.loginfo(object_pose)
        rospy.loginfo(corner11_pose)
        rospy.loginfo(corner12_pose)
        rospy.loginfo(corner21_pose)
        rospy.loginfo(corner22_pose)

        # print("before {}".format(object_pose))
        # object_pose = transform_pose(object_pose, "camera_link", "base_link")