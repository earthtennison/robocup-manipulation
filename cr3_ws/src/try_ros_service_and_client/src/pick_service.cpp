#include "ros/ros.h"
#include "try_ros_service_and_client/kan_pick.h"
#include <boost/filesystem.hpp>
#include <jsoncpp/json/json.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <tf2/LinearMath/Quaternion.h>
#include <vector>
//===============================================================
static const std::string PLANNING_GROUP_ARM = "arm";
static const std::string APP_DIRECTORY_NAME = ".cr3_simulation";
bool success = false;
//===============================================================
static const std::vector<double> OBJECT_POSITION = {0.5, 0, 0.5};
const double tau = 2 * M_PI;
//===============================================================

moveit_msgs::CollisionObject extractObstacleFromJson(Json::Value &root, std::string name){
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = name;

  const Json::Value dimensions = root["dimensions"];
  ROS_INFO_STREAM("Extracted dimensions: " << dimensions);
  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = dimensions["x"].asDouble();
  primitive.dimensions[1] = dimensions["y"].asDouble();
  primitive.dimensions[2] = dimensions["z"].asDouble();

  const Json::Value position = root["position"];
  ROS_INFO_STREAM("Extracted position: " << position);

  const Json::Value orientation = root["orientation"];
  ROS_INFO_STREAM("Extracted orientation: " << orientation);
  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = orientation["w"].asDouble();
  box_pose.orientation.x = orientation["x"].asDouble();
  box_pose.orientation.y = orientation["y"].asDouble();
  box_pose.orientation.z = orientation["z"].asDouble();

  // Moveit! planning scene expects the center of the object as position.
  // We add half of its dimension to its position
  box_pose.position.x = position["x"].asDouble() + primitive.dimensions[0] / 2.0;
  box_pose.position.y = position["y"].asDouble() + primitive.dimensions[1] / 2.0;
  box_pose.position.z = position["z"].asDouble() + primitive.dimensions[2] / 2.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return std::move(collision_object);  
}
// Generate Angle of Grasp Pose 
void roll_pitch_yawn(double** POSITION, int trial, double* OBJ_POS){
  float arr[7];
  // Algorithm from 0 +30 -30 +60 -60 +90 -90
  arr[0] = -float(M_PI)/2.0;
  arr[1] = -float(M_PI)/2.0 + float(M_PI)/6.0;
  arr[2] = -float(M_PI)/2.0 - float(M_PI)/6.0;
  arr[3] = -float(M_PI)/2.0 + float(M_PI)/3.0;
  arr[4] = -float(M_PI)/2.0 - float(M_PI)/3.0;
  arr[5] = -float(M_PI)/2.0 + float(M_PI)/2.0;
  arr[6] = -float(M_PI)/2.0 - float(M_PI)/2.0;
  for(int i = 0; i < 7; i++){
    POSITION[i][0] = OBJ_POS[0];
    POSITION[i][1] = OBJ_POS[1];
    POSITION[i][2] = OBJ_POS[2];
    POSITION[i][3] = -M_PI / 2.0;
    POSITION[i][4] = -M_PI / 4.0;
    POSITION[i][5] = arr[i];
  }
  ROS_INFO("Running roll pitch yawn");
}

// Void generate_pregrasp

// transform from roll pitch yaw coordinate to wxyz coordinate
// TODO ==> compute_grasp() --> transform_euler_to_wxyz
void compute_pregrasp(double** ORIENTATION, double** POSITION, int trial)
{
  for(int i = 0; i < 7; i++){
    ROS_INFO("Compute pregrasp");
    tf2::Quaternion quat0;
    quat0.setRPY(POSITION[i][3], POSITION[i][4], POSITION[i][5]);
    ORIENTATION[i][0] = quat0.getX();
    ORIENTATION[i][1] = quat0.getY();
    ORIENTATION[i][2] = quat0.getZ();
    ORIENTATION[i][3] = quat0.getW();
  }
}


void move(moveit::planning_interface::MoveGroupInterface &move_group_interface,
	  geometry_msgs::Pose goal_pose,
	  std::string str0, double** POSITION,
	  double** ORIENTATION, int trial){
  //Joint model group
  const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  //set target pose
  geometry_msgs::Pose target_pose1;
  int number = 0;
  success = false;
  ROS_INFO("Prepare to move");
  //random the target pose
  // to print an angle 
  int angle[7] = {0, 30, -30, 60, -60, 90, -90};
  while(number < trial){
    target_pose1.orientation.w = ORIENTATION[number][0];
    target_pose1.orientation.x = ORIENTATION[number][1];
    target_pose1.orientation.y = ORIENTATION[number][2];
    target_pose1.orientation.z = ORIENTATION[number][3];
    target_pose1.position.x = ORIENTATION[number][0];
    target_pose1.position.y = ORIENTATION[number][1];
    target_pose1.position.z = ORIENTATION[number][2];
    move_group_interface.setPoseTarget(target_pose1);
    ROS_INFO("target planning set order %d and %d degree done", number+1, angle[number]);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //=======================================================================================================//
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Plan %s", success ? "success" : "failure");
    //=======================================================================================================//
    //TODO = fill try catch instead 
    if(success == true){
      break;
    }
    number += 1;
  }
}

void move_catesian(moveit::planning_interface::MoveGroupInterface &move_group_interface, geometry_msgs::Pose current_pose, float x, float y, float z) {
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base link");
  visual_tools.deleteAllMarkers();

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose);

  geometry_msgs::Pose target_pose = current_pose;
  target_pose.position.x += x;
  target_pose.position.y += y;
  target_pose.position.z += z;
  waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  ROS_INFO_NAMED("tutorial", "visualizing plan 4 (cartesian path) (%.2f%% achieved)",fraction * 100.0);
  // visualize plan
  visual_tools.deleteAllMarkers();
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); i++){
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  }
  move_group_interface.execute(trajectory);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface) {
  // create vector to hold 3 object
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // table1
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  // table1 dimension
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  // table1 poses
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;

  collision_objects[0].operation = collision_objects[0].ADD;

  // object
  collision_objects[1].header.frame_id = "base_link";
  collision_objects[1].id = "object1";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.02;
  collision_objects[1].primitives[0].dimensions[1] = 0.02;
  collision_objects[1].primitives[0].dimensions[2] = 0.2;

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = OBJECT_POSITION[0];
  collision_objects[1].primitive_poses[0].position.y = OBJECT_POSITION[1];
  collision_objects[1].primitive_poses[0].position.z = OBJECT_POSITION[2];

  collision_objects[1].operation = collision_objects[1].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}
// void planning_pose(float*** GRASP_GEN, int trial, int stepp, geometry_msgs::Pose pose, tf2::Quaternion quat){
//   pose.position.x = GRASP_GEN[trial][stepp][0];
//   pose.position.y = GRASP_GEN[trial][stepp][1];
//   pose.position.z = GRASP_GEN[trial][stepp][2];
//   quat.setRPY(GRASP_GEN[trial][stepp][3], GRASP_GEN[trial][stepp][4], GRASP_GEN[trial][stepp][5]);
//   pose.orientation.x = quat.getX();
//   pose.orientation.y = quat.getY();
//   pose.orientation.z = quat.getZ();
//   pose.orientation.w = quat.getW();
// }

bool kan_grasp_server(try_ros_service_and_client::kan_pick::Request &req,
	 try_ros_service_and_client::kan_pick::Response &res)
{
  namespace fs = boost::filesystem;
  double pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w;
  pos_x = req.geo_req.position.x;
  pos_y = req.geo_req.position.y;
  pos_z = req.geo_req.position.z;
  ori_x = req.geo_req.orientation.x;
  ori_y = req.geo_req.orientation.y;
  ori_z = req.geo_req.orientation.z;
  ori_w = req.geo_req.orientation.w;
  ROS_INFO("request: x=%lf, y=%lf, z=%lf, ox=%lf, oy=%lf, oz=%lf, ow=%lf", pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w);
  //Put the procedure
  
  ROS_INFO("RUNNING robot_control_node");
  ros::NodeHandle nh1;
  ROS_INFO("Node Handle");
  ros::Publisher gripper_command_publisher = nh1.advertise<std_msgs::Bool>("/gripper_command",10);
  ROS_INFO("Gripper command publisher");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("dusfahoaeroh");
  moveit::planning_interface::MoveGroupInterface move_group_arm(PLANNING_GROUP_ARM);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  ros::Publisher planning_scene_diff_publisher = nh1.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  // declare interface for moveit
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  // rviz visual and moveit visual
  moveit_msgs::PlanningScene planning_scene;
  //
  // read JSON files from ~/.cr3_simulation
  fs::path home(getenv("HOME"));
  fs::path app_directory(home);
  if (fs::is_directory(home)){
    app_directory /= APP_DIRECTORY_NAME;
    if (!fs::exists(app_directory) && !fs::is_directory(app_directory)){
      ROS_WARN_STREAM(app_directory << " does not exist");
      // Create .cr3_simulation directory
      std::string path(getenv("HOME"));
      path += "/.cr3_simulation";
      ROS_INFO("Creating %s collision objects directory.", path);
      try{
        boost::filesystem::create_directory(path);
      }
      catch (const std::exception &){
        ROS_ERROR("%s directory could not be created."
                  "Please create this directory yourself "
                  "if you want to specify collision objects.",
                  path.c_str());
        return -1;
      }
    }
  }

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  ROS_INFO_STREAM(app_directory << " is a directory containing:");
  for (auto &entry : boost::make_iterator_range(fs::directory_iterator(app_directory), {})){
    ROS_INFO_STREAM(entry);
    std::ifstream file_stream(entry.path().string(), std::ifstream::binary);
    if (file_stream){
      Json::Value root;
      file_stream >> root;
      moveit_msgs::CollisionObject collision_object = extractObstacleFromJson(root, entry.path().stem().string());
      collision_objects.push_back(collision_object);
    }
    else{
      ROS_WARN_STREAM("could not open file " << entry.path());
    }
  }

    // Publish the collision objects to the scene
  for (const auto &collision_object : collision_objects){
      collision_object.header.frame_id = move_group_arm.getPlanningFrame();
      planning_scene.world.collision_objects.push_back(collision_object);
  }
  ROS_INFO_STREAM("# collision objects " << planning_scene.world.collision_objects.size());
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  ROS_INFO("robot_control_node is ready");
  
    // visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to add collision object");

  /////////////// planning scene interface /////////////

  // addCollisionObjects(planning_scene_interface);
  
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to pregrasp");
  
  double* OBJ_POS;
  OBJ_POS = (double *) malloc (3 * sizeof(double));
  OBJ_POS[0] = pos_x;
  OBJ_POS[1] = pos_y;
  OBJ_POS[2] = pos_z;
  int number_of_repeat = 7;
  int number_of_grasp = 5;
  double **POSITION;
  POSITION = (double **) malloc (number_of_repeat * sizeof(double));
  for(int i = 0; i < number_of_repeat; i++){
    POSITION[i] = (double *) malloc (6 * sizeof(double));
  }

  double **ORIENTATION;
  ORIENTATION = (double **) malloc (number_of_repeat * sizeof(double));
  for(int i = 0; i < number_of_repeat; i++){
    ORIENTATION[i] = (double *) malloc (4 * sizeof(double));
  }

  roll_pitch_yawn(POSITION, number_of_repeat, OBJ_POS);
  //add the grasp gen function of the OBjECT_POSITION in the pointer
  //declare the pose function;
  geometry_msgs::Pose poseA;
  tf2::Quaternion quatA;
  move(move_group_arm, poseA, "string", POSITION, ORIENTATION, number_of_repeat);
  //check whether it is true
  res.success_grasp = success;
  ROS_INFO(res.success_grasp ? "true" : "false");
  return true;
}



int main(int argc, char **argv){
  
  ros::init(argc, argv, "pick_success_server");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("pick_success", kan_grasp_server);

  ////////////// move group interface /////////////////
  
  // ros::WallDuration(3.0).sleep();

  // pregrasp
  // the position for panda_link8 = object pose - (length of cube/2 - distance b/w panda_link8 and palm of eef (0.058)
  // - some extra padding) - (desired offset for pregasp)
  // add the pointer for the convinience for calculation
  ROS_INFO("Ready to add six parameter");
  ros::spin();
  return 0;
}
