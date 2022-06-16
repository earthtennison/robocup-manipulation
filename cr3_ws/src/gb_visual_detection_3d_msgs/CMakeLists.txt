cmake_minimum_required(VERSION 2.8.3)
project(gb_visual_detection_3d_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  darknet_ros_msgs
  message_generation
  
)

add_message_files(
 FILES
 BoundingBoxes3d.msg
 BoundingBox3d.msg
)

add_service_files(
  FILES
  Detect3d.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
 sensor_msgs
 darknet_ros_msgs

)
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  sensor_msgs
  darknet_ros_msgs
)
