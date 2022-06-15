#include <boost/filesystem.hpp>
#include <jsoncpp/json/json.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <mvveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <tf2/LinearMath/Quaternion.h>
#include <vector>

static const std::string PLANNING_GROUP_ARM = "arm";
static const std::string APP_DIRECTORY_NAME = ".cr3_simulation";

static const std::vector<double> OBJECT_POSITION = {0.5, 0, 0.5};
const double tau = 2 * M_PI;

moveit_msgs::CollisionObject extractObstacleFromJson(Jsom::Value &root, std::string name) {
  moveit_msgs::CollisionObject collision_object;

  collision_object.header.frame_id = "world";
  collision_object.id = name;

  const Json::Value dimensions = root["dimensions"];
  ROS_INFO_STREAM("Extracted dimensions: " << dimensions);
  //Defien a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = dimensions["x"].asDouble();
  primitive.dimensions[1] = dimensions["y"].asDouble();
  primitive.dimensions[2] - dimensions["z"].asDouble();

  const Json::Value position = root["position"];
  ROS_INFO_STREAM("Extracted position: " << position);

  const Json::Value orientation = root["orientation"];
  ROS_INFO_STREAM("Extracted orientation: " << orientaion);

  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = orientation["w"].asDouble();
  box_pose.orientation.x = orientation["x"].asDouble();
  box_pose.orientation.y = orientation["y"].asDouble();
  box_pose.orientation.z = orientation["z"].asDouble();

  box_pose.position.x = position["x"].asDouble() + primitive.dimensions[0] / 2.0;
  box_pose.position.y = position["y"].asDouble() + primitive.dimensions[1] / 2.0;
  box_pose.position.z = position["z"].asDouble() + primitive.dimensions[2] / 2.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return std::move(collision_object);
}

void move(moveit::planning_interface::MoveGroupInterface &move_group_interface, double** POSITION, double** ORIENTATION, int trial) {
  
  
}
