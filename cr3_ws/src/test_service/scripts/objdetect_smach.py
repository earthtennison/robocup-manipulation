#!/usr/bin/env python

"""
roslaunch cr3_moveit_control manipulation_pipeline_real.launch
"""

import roslib
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math
import time

# pick service
from cr3_moveit_control.srv import PickWithSide

# realsense
import pyrealsense2 as rs2
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# # tf for visualization
import tf2_msgs
import tf
from geometry_msgs.msg import TransformStamped

# # computer vision
# import socket
from custom_socket import CustomSocket
import numpy as np

# # output
from geometry_msgs.msg import Pose

# transform to from cameralink to base link
import tf2_ros
import tf2_geometry_msgs


class GetObjectName(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectName')
        smach.State.__init__(self, outcomes=['continue_GetObjectPose'], output_keys=[
                             'objectname_output'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectName')
        # sending object name to GetobjectName state (change string right here)
        userdata.objectname_output = "Waterbottle"
        return 'continue_GetObjectPose'


class GetObjectPose(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectPose')
        smach.State.__init__(self, outcomes=['continue_GetProperties', 'continue_ABORTED'], input_keys=[
                             'objectname_input', 'ListBBX_output'], output_keys=['ListBBX_output'])
        # initiate variables
        self.object_name = ""
        self.bbx_pixel_list = [] # [(xm1, ym1, xM1, yM1, id), (xm2, ym2, xM2, yM2, id), ...] in pixels
        self.intrinsics = None
        self.bridge = CvBridge()
        self.frame = None
        self.tf_stamp = None

        # connect to CV server
        host = "192.168.8.99"
        port = 10001
        self.c = CustomSocket(host, port)
        self.c.clientConnect()
        rospy.loginfo("connected object detection server")

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectPose')

        def run_once():
            while self.intrinsics is None:
                time.sleep(0.1)
            rospy.loginfo("realsense image width, height = ({}, {})".format(self.intrinsics.width, self.intrinsics.height))
            self.c.req(np.random.randint(255, size=(720, 1280, 3), dtype=np.uint8))

        def reset():
            rospy.loginfo("Reseting the value")
            self.frame = None
            rospy.sleep(0.1)
            rospy.loginfo("Finished reseting")
            self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, yolo_callback, queue_size=1, buff_size=52428800)
            self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1, buff_size=52428800)

        def detect():
            rospy.loginfo("Start detecting")
            # send frame to server and recieve the result
            result = self.c.req(self.frame)
            self.frame = check_image_size_for_ros(self.frame)
            rospy.loginfo("result {}".format(result))
            if result['n'] == 0:
                return None 
            # object detection bounding box 2d
            for bbox in result['bbox_list']:
                if bbox[4] != self.object_name:
                    continue
                else:
                    # receive xyxy                
                    # (xmin,ymin)----------*
                    # |                    |
                    # |                    |
                    # |                    |
                    # |                    |
                    # *----------(xmax,ymax)
                    # Get List of Bounding Box in Pixel

                    (xmin_pixel,ymin_pixel) = rescale_pixel(bbox[0], bbox[1])
                    (xmax_pixel,ymax_pixel) = rescale_pixel(bbox[2], bbox[3])
                    object_id = 1 # TODO change to object tracker

                    self.bbx_pixel_list.append(xmin_pixel,ymin_pixel,xmax_pixel,ymax_pixel, object_id)

                    # visualize purpose
                    x_pixel = int(bbox[0] + (bbox[2]-bbox[0])/2)
                    y_pixel = int(bbox[1] + (bbox[3]-bbox[1])/2)
                    (x_pixel, y_pixel) = rescale_pixel(x_pixel, y_pixel)
                    
                    self.frame = cv2.circle(self.frame, (x_pixel, y_pixel), 5, (0, 255, 0), 2)
                    self.frame = cv2.rectangle(self.frame, (xmin_pixel,ymin_pixel), (xmax_pixel,ymax_pixel), (0, 0, 255), 2)
                                    
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))

            self.image_sub.unregister()
            self.depth_sub.unregister()

        # function used in callback functions
        def check_image_size_for_cv(frame):
            if frame.shape[0] != 720 and frame.shape[1] != 1280:
                frame = cv2.resize(frame, (1280, 720))
            return frame

        def check_image_size_for_ros(frame):
            if frame.shape[0] != self.intrinsics.height and frame.shape[1] != self.intrinsics.width:
                frame = cv2.resize(frame, (self.intrinsics.width, self.intrinsics.height))
            return frame

        def rescale_pixel(x, y):
            x = int(x*self.intrinsics.width/1280)
            y = int(y*self.intrinsics.height/720)
            return (x, y)

        # all call_back functions
        def info_callback(cameraInfo):
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

        def yolo_callback(data):
            try:
                # change subscribed data to numpy.array and save it as "frame"
                self.frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
                # scale image incase image size donot match cv server
                self.frame = check_image_size_for_cv(self.frame)
            except CvBridgeError as e:
                print(e)

        def depth_callback(frame):
            try:
                if self.tf_stamp is not None:
                    # rospy.loginfo("publishing tf")
                    self.tf_stamp.header.stamp = rospy.Time.now()
                    self.pub_tf.publish(tf2_msgs.msg.TFMessage([self.tf_stamp]))

                self.depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
                # rescale pixel incase pixel donot match
                self.depth_image = check_image_size_for_ros(self.depth_image)

            except CvBridgeError as e:
                print(e)
                return
            except ValueError as e:
                return
            pass

        # ----------------------------------------------start-----------------------------------------------------
        # subscribe topics
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info", CameraInfo, info_callback)
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, yolo_callback, queue_size=1, buff_size=52428800)
        self.depth_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1, buff_size=52428800)
        self.image_pub = rospy.Publisher(
            "/blob/image_blob", Image, queue_size=1)
        

        # recieving object name from GetObjectName state
        self.object_name = userdata.objectname_input
        rospy.loginfo(self.object_name)

        # run_once function
        run_once()
        while not rospy.is_shutdown():
            command = raw_input("Press Enter :")
            if command == 'q':
                break
            rospy.loginfo("------ Running 3D detection ------")
            reset()
            detect()
            userdata.ListBBX_output = self.bbx_pixel_list
            return 'continue_GetProperties'
        return 'continue_ABORTED'

class GetObjectProperties():
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectProperties')
        smach.State.__init__(self, outcomes=['continue_Place', 'continue_ABORTED'], input_keys=[
                             'ListBBX_input'], output_keys=['ObjectProperties_output'])
        # initiate variables
        self.object_name = ""
        self.center_pixel_list = [] # [(x1, y1, id), (x2, y2, id), ...] in pixels
        self.object_pose_list = [] # [(x1, y1, z1, id), (x1, y1, z1, id), ...] im meters
        self.intrinsics = None
        self.bridge = CvBridge()
        self.frame = None
        self.object_pose = Pose()
        self.tf_stamp = None
    
    def execute(self, userdata):
        pass

def main():
    rospy.init_node('smach_pick_state_machine')
    sm = smach.StateMachine(outcomes=['SUCCEEDED', 'ABORTED'])
    sm.userdata.string_name = ""
    sm.userdata.object_pose = Pose()
    sm.userdata.bbx_list = []
    with sm:
        smach.StateMachine.add('GetObjectName', GetObjectName(),
                               transitions={
                                   'continue_GetObjectPose': 'GetObjectPose'},
                               remapping={'objectname_output': 'string_name'})
        smach.StateMachine.add('GetObjectPose', GetObjectPose(),
                               transitions={'continue_GetProperties': 'GetObjectProperties',
                                            'continue_ABORTED': 'ABORTED'},
                               remapping={'objectname_input': 'string_name',
                                          'ListBBX_output': 'bbx_list',
                                          'ListBBX_output': 'bbx_list'})
        # smach.StateMachine.add('Pick', Pick(),
        #                        transitions={'continue_SUCCEEDED': 'SUCCEEDED',
        #                                     'continue_ABORTED': 'ABORTED'},
        #                        remapping={'objectpose_input': 'object_pose'})
    outcome = sm.execute()


if __name__ == "__main__":
    main()
