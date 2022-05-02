cmake_minimum_required(VERSION 3.1.3)
project(cr3_gripper_pkg)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp 
  std_msgs
  actionlib
  control_msgs
  trajectory_msgs
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  # LIBRARIES
  # INCLUDE_DIRS
  # CATKIN_DEPENDS
  # DEPENDS
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})


###########
## Build ##
###########

add_executable(simple_gripper_control src/simple_gripper_control.cpp)
target_link_libraries(simple_gripper_control ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

install(TARGETS simple_gripper_control DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

