#include "cr3_moveit_control/pick_gen2.h"
#include "ros/ros.h"
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
//include quaternian transformation
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <vector>
//===============================================================
static const std::string PLANNING_GROUP_ARM = "arm";
bool success = false;

ros::Publisher gripper_command_publisher;
//===============================================================
//========================================================================================
//========================================================================================
//===========================================================================================
//===========================================================================================


//Move the arm

void move(geometry_msgs::Pose goal_pose) {
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  // set target pose
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = goal_pose.orientation.w;
  target_pose.orientation.x = goal_pose.orientation.x;
  target_pose.orientation.y = goal_pose.orientation.y;
  target_pose.orientation.z = goal_pose.orientation.z;
  target_pose.position.x = goal_pose.position.x;
  target_pose.position.y = goal_pose.position.y;
  target_pose.position.z = goal_pose.position.z;

  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan %s", success ? "success" : "failure");

  // visualize plan
  visual_tools.publishAxisLabeled(target_pose, "pose 1");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  bool success_execute =  (move_group_interface.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute %s", success_execute ? "success" : "failure");
  success = success && success_execute;
}
//Pick execution
bool pick_server2(cr3_moveit_control::pick_gen2::Request &req,
		      cr3_moveit_control::pick_gen2::Response &res){
  double pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w;
  pos_x = req.geo_req.position.x;
  pos_y = req.geo_req.position.y;
  pos_z = req.geo_req.position.z;
  ori_x = req.geo_req.orientation.x;
  ori_y = req.geo_req.orientation.y;
  ori_z = req.geo_req.orientation.z;
  ori_w = req.geo_req.orientation.w;
  std::string side;
  side = req.pick_side;
  ROS_INFO("request: x=%lf, y=%lf, z=%lf, ox=%lf, oy=%lf, oz=%lf, ow=%lf", pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w);
  //ROS_INFO("the side to pick up: side=%s",side);
  // initialize the quaternion angle 
  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::convert(req.geo_req.orientation, q_orig); // calcultate the q_orig from the requested geo_request orientation.
  // Pregrasp
  geometry_msgs::Pose pose;
  std::vector<geometry_msgs::Pose> pose_array = {};
  if(side == "front"){
    //Cartesian coordinate
    pose.position.x = pos_x + 0.1; 
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    //Calculate angle
    q_rot.setRPY(0,0,0);
    q_new = q_rot * q_orig;
    q_new.normalize();
    geometry_msgs::Pose new_pose = pose;
    tf2::convert(q_new, new_pose.orientation);
    pose_array.push_back(new_pose);
    pose.orientation.x = new_pose.orientation.x;
    pose.orientation.y = new_pose.orientation.y;
    pose.orientation.z = new_pose.orientation.z;
    pose.orientation.w = new_pose.orientation.w;
    move(pose);
    res.success_grasp = success;
    ROS_INFO(res.success_grasp ? "true" : "false");
    if(!success){
        return false;
    }
    else{
        return true;
    }
  }
  else if(side == "top"){
    //Cartesian coordinate
    pose.position.x = pos_x; 
    pose.position.y = pos_y;
    pose.position.z = pos_z + 0.1;
    //Calculate angle
    q_rot.setRPY(0,- M_PI / 2.0,0);
    q_new = q_rot * q_orig;
    q_new.normalize();
    geometry_msgs::Pose new_pose = pose;
    tf2::convert(q_new, new_pose.orientation);
    pose_array.push_back(new_pose);
    pose.orientation.x = new_pose.orientation.x;
    pose.orientation.y = new_pose.orientation.y;
    pose.orientation.z = new_pose.orientation.z;
    pose.orientation.w = new_pose.orientation.w;
    move(pose);
    res.success_grasp = success;
    ROS_INFO(res.success_grasp ? "true" : "false");
    if(!success){
        return false;
    }
    else{
        return true;
    }
  }
  else if(side == "left"){
    //Cartesian coordinate
    pose.position.x = pos_x; 
    pose.position.y = pos_y - 0.1;
    pose.position.z = pos_z;
    //Calculate angle    
    q_rot.setRPY(0,0,-M_PI / 2.0);
    q_new = q_rot * q_orig;
    q_new.normalize();
    geometry_msgs::Pose new_pose = pose;
    tf2::convert(q_new, new_pose.orientation);
    pose_array.push_back(new_pose);
    pose.orientation.x = new_pose.orientation.x;
    pose.orientation.y = new_pose.orientation.y;
    pose.orientation.z = new_pose.orientation.z;
    pose.orientation.w = new_pose.orientation.w;
    move(pose);
    res.success_grasp = success;
    ROS_INFO(res.success_grasp ? "true" : "false");
    if(!success){
        return false;
    }
    else{
        return true;
    }
  }
  else if(side == "right"){
    //Cartesian coordinate
    pose.position.x = pos_x; 
    pose.position.y = pos_y + 0.1;
    pose.position.z = pos_z;
    //Calculate angle
    q_rot.setRPY(0,0,M_PI / 2.0);
    q_new = q_rot * q_orig;
    q_new.normalize();
    geometry_msgs::Pose new_pose = pose;
    tf2::convert(q_new, new_pose.orientation);
    pose_array.push_back(new_pose);
    pose.orientation.x = new_pose.orientation.x;
    pose.orientation.y = new_pose.orientation.y;
    pose.orientation.z = new_pose.orientation.z;
    pose.orientation.w = new_pose.orientation.w;
    move(pose);
    res.success_grasp = success;
    ROS_INFO(res.success_grasp ? "true" : "false");
    if(!success){
        return false;
    }
    else{
        return true;
    }
  }
  else{
    return false;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_gen2_success_server");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("pick_gen2_success", pick_server2);
  ros::waitForShutdown();
  return 0;
}
