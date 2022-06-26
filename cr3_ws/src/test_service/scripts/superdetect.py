#!/usr/bin/env python

import rospy
import time

#msg
from geometry_msgs.msg import Pose

#realsense
from sensor_msgs.msg import Image, CameraInfo
import pyrealsense2 as rs2
import cv2
from cv_bridge import CvBridge, CvBridgeError

#rviz msg
from visualization_msgs.msg import Marker

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

class GetObjectProperties():
    pass