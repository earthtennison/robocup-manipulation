cmake_minimum_required(VERSION 3.1.3)
project(moveit)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

###########
## Build ##
###########

add_executable(move_group_interface src/move_group_interface.cpp)
target_link_libraries(move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(move_group_interface_demo src/move_group_interface_demo.cpp)
target_link_libraries(move_group_interface_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(pick_and_place src/pick_and_place.cpp)
target_link_libraries(pick_and_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(pick_and_place_ball src/pick_and_place_ball.cpp)
target_link_libraries(pick_and_place_ball ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_install_python(PROGRAMS
  src/cv_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Install ##
#############

install(TARGETS move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

