#include "ros/ros.h"
#include "cr3_moveit_control/cr3_place.h"
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

#include <stdio.h>
#include <algorithm>

//===============================================================
static const std::string PLANNING_GROUP_ARM = "arm";
bool success = false;

ros::Publisher gripper_command_publisher;
//===============================================================

float to_rad(float deg)
{
  return deg * M_PI / 180.0;
}

void move(geometry_msgs::Pose POSITION) {
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  geometry_msgs::Pose target_pose1;
  //Step1 Execute along X-Y coordinate
  bool success = false;
  ROS_INFO("INIITIATED STEP1");

  //construct quaternion angle
  tf2::Quaternion quat;
  quat.setRPY(-M_PI / 2, -M_PI / 4, M_PI / 2);
  target_pose1.orientation.w = quat.getW();
  target_pose1.orientation.x = quat.getX();
  target_pose1.orientation.y = quat.getY();
  target_pose1.orientation.z = quat.getZ();
  target_pose1.position.x = POSITION.position.x;
  target_pose1.position.y = POSITION.position.y;
  target_pose1.position.z = POSITION.position.z;
  
  move_group_interface.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan %s", success ? "success" : "failure");
  visual_tools.publishAxisLabeled(target_pose1, "pose 1");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  bool success_execute = (move_group_interface.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute %s", success_execute ? "success" : "failure");
  success = success && success_execute;
}

void move_cartesian(geometry_msgs::Pose current_pose, float x, float y, float z) {
  namespace rvt = rviz_visual_tools;
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
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

  // visualize plan
  visual_tools.deleteAllMarkers();
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);

  bool success_execute = (move_group_interface.execute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute %s", success_execute ? "success" : "failure");
  success = success_execute;
}

void set_home_walkie2(void)
{
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = 0.0;  // radians
  joint_group_positions[1] = 0.0;  // radians
  joint_group_positions[2] = 2.267;  // radians
  joint_group_positions[3] = 0.875;  // radians
  // joint_group_positions[4] = 1.507;  // radians
  joint_group_positions[4] = 0.0;  // radians
  joint_group_positions[5] = 2.355;  // radians

  move_group_interface.setJointValueTarget(joint_group_positions);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan %s", success ? "success" : "failure");

  // visualize plan
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  bool success_execute = 
        (move_group_interface.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute %s", success_execute ? "success" : "failure");
  success = success && success_execute;
}

void addCollisionTable( float dimension_x, float dimension_y, float dimension_z,
                        float position_x, float position_y, float position_z )
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // create vector to hold 3 object
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // collision_table
  collision_objects[0].id = "collision_table";
  collision_objects[0].header.frame_id = "base_link";

  // collsion_table dimension
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = dimension_x;
  collision_objects[0].primitives[0].dimensions[1] = dimension_y;
  collision_objects[0].primitives[0].dimensions[2] = dimension_z;

  // collsion_table poses
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = position_x;
  collision_objects[0].primitive_poses[0].position.y = position_y;
  collision_objects[0].primitive_poses[0].position.z = position_z;

  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

bool place_server(cr3_moveit_control::cr3_place::Request &req,
                      cr3_moveit_control::cr3_place::Response &res)
{
  set_home_walkie2();

  // calculate collision table and space to place
  // *table11-----------------------------table12
  // |                                          |
  // |                                          |
  // |   place11----------------------*place12  |
  // |   |                                  |   |
  // |   |                                  |   |
  // |   |                                  |   |
  // |   |                                  |   |
  // |   *place12----------------------place22  |
  // |                                          |
  // |                                          |
  // table21-----------------------------*table22
  //
  //                     z ---> y
  //                     | 
  //                     v
  //                     x
    //x axis
    //condition --> corner.x are in -x axis 
  double creat_collision_table11_x, creat_collision_table12_x, creat_collision_table21_x, creat_collision_table22_x;
  double place_within11_x, place_within12_x, place_within21_x, place_within22_x;

  double calculate_collision_table_x[4] = {req.corner11.x, req.corner12.x, req.corner21.x, req.corner22.x};
  std::sort(calculate_collision_table_x, calculate_collision_table_x + 4);
  creat_collision_table11_x = calculate_collision_table_x[0];
  creat_collision_table12_x = calculate_collision_table_x[0];
  creat_collision_table21_x = calculate_collision_table_x[3];
  creat_collision_table22_x = calculate_collision_table_x[3];

  place_within11_x = calculate_collision_table_x[1];
  place_within12_x = calculate_collision_table_x[1];
  place_within21_x = calculate_collision_table_x[2];
  place_within22_x = calculate_collision_table_x[2];

    //y axis
  double creat_collision_table11_y, creat_collision_table12_y, creat_collision_table21_y, creat_collision_table22_y;
  double place_within11_y, place_within12_y, place_within21_y, place_within22_y;

  double calculate_collision_table_y[4] = {req.corner11.y, req.corner12.y, req.corner21.y, req.corner22.y};
  std::sort(calculate_collision_table_y, calculate_collision_table_y + 4);
  creat_collision_table11_y = calculate_collision_table_y[0];
  creat_collision_table12_y = calculate_collision_table_y[3];
  creat_collision_table21_y = calculate_collision_table_y[0];
  creat_collision_table22_y = calculate_collision_table_y[3];

  place_within11_y = calculate_collision_table_y[1];
  place_within12_y = calculate_collision_table_y[2];
  place_within21_y = calculate_collision_table_y[1];
  place_within22_y = calculate_collision_table_y[2];

  // printf("x axis\n");
  // printf("%f          %f\n",creat_collision_table11_x, creat_collision_table12_x);
  // printf("%f          %f\n",creat_collision_table21_x, creat_collision_table22_x);
  
  // printf("%f          %f\n",place_within11_x, place_within12_x);
  // printf("%f          %f\n\n\n",place_within21_x, place_within22_x);

  // printf("y axis\n");
  // printf("%f          %f\n",creat_collision_table11_y, creat_collision_table12_y);
  // printf("%f          %f\n",creat_collision_table21_y, creat_collision_table22_y);
  
  // printf("%f          %f\n",place_within11_y, place_within12_y);
  // printf("%f          %f\n\n\n",place_within21_y, place_within22_y);

    //z axis
    double high;
    high = req.high.z;

  /////////////// add collsion table /////////////
  double dimension_x, dimension_y, dimension_z;
  double position_x, position_y, position_z;

  dimension_x = abs(creat_collision_table11_x - creat_collision_table21_x);
  dimension_y = abs(creat_collision_table11_y - creat_collision_table12_y);
  dimension_z = high;

  position_x = (creat_collision_table11_x + creat_collision_table21_x) / 2;
  position_y = (creat_collision_table21_y + creat_collision_table22_y) / 2;
  position_z = dimension_z / 2;

  addCollisionTable(dimension_x, dimension_y, dimension_z
                  , position_x, position_y, position_z );

  //find pose to preplace
  geometry_msgs::Pose pose;
  tf2::Quaternion myQuaternion;

  pose.position.x = place_within21_x - 0.1;
  pose.position.y = (place_within21_y + place_within22_y) / 2;
  pose.position.z = high + 0.15;

  myQuaternion.setRPY( -1 * M_PI / 2, -1 * M_PI / 4, M_PI / 2 );
  pose.orientation.x = myQuaternion.getX();
  pose.orientation.y = myQuaternion.getY();
  pose.orientation.z = myQuaternion.getZ();
  pose.orientation.w = myQuaternion.getW();
  
  ROS_INFO_STREAM(pose);
  
  //move to preplace pose
  move(pose);
  if (!success){
    return false;
  }

  //move cartesian along z axis
  move_cartesian(pose, 0, 0, -0.1);
  pose.position.z -= 0.1;
  if (!success){
    return false;
  }

  // open gripper
  std_msgs::Bool gripper_command_msg;
  gripper_command_msg.data = false;
  gripper_command_publisher.publish(gripper_command_msg);

  //move cartesian out of object
  move_cartesian(pose, 0.1, 0, 0.1);
  if (!success){
    return false;
  }
  pose.position.x += 0.1;
  pose.position.z += 0.1;

  // close gripper
  gripper_command_msg.data = true;
  gripper_command_publisher.publish(gripper_command_msg);

  // set home walkie2
  set_home_walkie2();
  

  // check whether it is true then return "success value"
  res.success_place = success;
  ROS_INFO(res.success_place ? "true" : "false");
  return true;
}

int main(int argc, char** argv){

  ros::init(argc, argv, "place_service_server");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh;
  gripper_command_publisher = nh.advertise<std_msgs::Bool>("/cr3_gripper_command", 10);
  ros::ServiceServer service = nh.advertiseService("cr3_place", place_server);

  // // place_client_python //////////////////////////
  
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // cr3_moveit_control::cr3_place::Request &req;
  // cr3_moveit_control::cr3_place::Response &res;

  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // float corner11_x, corner12_x, corner21_x, corner22_x,
  //       corner11_y, corner12_y, corner21_y, corner22_y,
  //       high,
  //       dimension_x, dimension_y, dimension_z,
  //       position_x, position_y, position_z;

  // corner11_x = req.corner11.x;
  // corner12_x = req.corner12.x;
  // corner21_x = req.corner21.x;
  // corner22_x = req.corner22.x;

  // corner11_y = req.corner11.y;
  // corner12_y = req.corner12.y;
  // corner21_y = req.corner21.y;
  // corner22_y = req.corner22.y;

  // high = req.high.x;

  // corner11_x = -1.5;
  // corner12_x = -1.5;
  // corner21_x = -0.5;
  // corner22_x = -0.5;

  // corner11_y = -1.0;
  // corner12_y = 1.0;
  // corner21_y = -1.0;
  // corner22_y = 1.0;

  // high = 0.15;

  // dimension_x = abs(corner11_x - corner21_x);
  // dimension_y = abs(corner21_y - corner22_y);
  // dimension_z = 0.15;

  // position_x = (corner11_x + corner21_x) / 2;
  // position_y = (corner21_y + corner22_y) / 2;
  // position_z = dimension_z / 2;

  // addCollisionTable(dimension_x
  //                 , dimension_y
  //                 , dimension_z
  //                 , position_x
  //                 , position_y
  //                 , position_z );

  // // //preplace
  // geometry_msgs::Pose POSITION;
  // POSITION.position.x = -0.5;
  // POSITION.position.y = -0.2;
  // POSITION.position.z = 0.20;

  // kan_place(POSITION);
  
  // moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);

  // // Raw pointers are frequently used to refer to the planning group for improved performance.
  // const robot_state::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  // // Visualization
  // // ^^^^^^^^^^^^^
  // //
  // // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  // namespace rvt = rviz_visual_tools;
  // visual_tools.deleteAllMarkers();

  // // Remote control is an introspection tool that allows users to step through a high level script
  // // via buttons and keyboard shortcuts in RViz
  // visual_tools.loadRemoteControl();

  // // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  // Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  // text_pose.translation().z() = 1.75;
  // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  // visual_tools.trigger();

  // // Getting Basic Information
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // //
  // // We can print the name of the reference frame for this robot.
  // ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // // We can also print the name of the end-effector link for this group.
  // ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // // We can get a list of all the groups in the robot:
  // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  // std::copy(move_group_interface.getJointModelGroupNames().begin(), move_group_interface.getJointModelGroupNames().end(),
  //           std::ostream_iterator<std::string>(std::cout, ", "));

  // // Start the demo
  // // ^^^^^^^^^^^^^^^^^^^^^^^^^
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // // .. _move_group_interface-planning-to-pose-goal:
  // //
  // // Planning to a Pose goal
  // // ^^^^^^^^^^^^^^^^^^^^^^^
  // // We can plan a motion for this group to a desired pose for the
  // // end-effector.
  // geometry_msgs::Pose target_pose1;
  // target_pose1.orientation.w = 1.0;
  // target_pose1.position.x = -0.4;
  // target_pose1.position.y = 0.2;
  // target_pose1.position.z = 0.4;
  // move_group_interface.setPoseTarget(target_pose1);

  // // Now, we call the planner to compute the plan and visualize it.
  // // Note that we are just planning, not asking move_group
  // // to actually move the robot.
  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // set_home_walkie2();
  
  /////////////////////////////////////////////////
  ros::waitForShutdown();
  return 0;
}