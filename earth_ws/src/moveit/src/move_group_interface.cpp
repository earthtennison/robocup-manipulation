#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

const double tau = 2 * M_PI;

class MyMoveGroupInterface
{
private:
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::NodeHandle node_handle;
  std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface* move_group_interface_ptr;
  moveit_visual_tools::MoveItVisualTools* visual_tools_ptr;
  moveit::core::JointModelGroup* joint_model_group;

public:
  MyMoveGroupInterface()
  {
  // initialize node
  namespace rvt = rviz_visual_tools;
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  move_group_interface_ptr = &move_group_interface;
  joint_model_group = move_group_interface_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools_ptr = &visual_tools;

  sub = node_handle.subscribe<geometry_msgs::Pose>("object_coor", 10, &MyMoveGroupInterface::object_coor_callback, this);
  }

  void object_coor_callback(const geometry_msgs::Pose::ConstPtr& msg)
  {
      ROS_INFO("[CV_NODE]: x: [%f] y:[%f] z: [%f]", msg->position.x, msg->position.y, msg->position.z);
      ROS_INFO("[CV_NODE]: qx: [%f] qy:[%f] qz: [%f] qw:[%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
      
      // set goal pose
      geometry_msgs::Pose target_pose1;
      target_pose1.orientation.w = msg->orientation.w;
      target_pose1.orientation.x = msg->orientation.x;
      target_pose1.orientation.y = msg->orientation.y;
      target_pose1.orientation.z = msg->orientation.z;
      target_pose1.position.x = msg->position.x;
      target_pose1.position.y = msg->position.y;
      target_pose1.position.z = msg->position.z;

      move_group_interface_ptr->setPoseTarget(target_pose1);

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_interface_ptr->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Plan %s", success ? "success" : "failure");

      // visualize plan
      visual_tools_ptr->publishAxisLabeled(target_pose1, "pose 1");
      visual_tools_ptr->publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools_ptr->trigger();

      bool success_execute = (move_group_interface_ptr->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Plan %s", success_execute ? "success" : "failure");
      ROS_INFO("Successfully executed!");
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  
  //setup visualizer
  MyMoveGroupInterface my_move;
  ros::waitForShutdown();
  return 0;
}