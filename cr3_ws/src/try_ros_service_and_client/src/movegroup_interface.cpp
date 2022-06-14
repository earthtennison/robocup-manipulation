
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
  geometry_msgs::Pose::ConstPtr goal_pose;

public:
  MyMoveGroupInterface()
  {
  // initialize node
  sub = node_handle.subscribe("object_coor", 10, &MyMoveGroupInterface::object_coor_callback, this);
  
  }

  void object_coor_callback(const geometry_msgs::Pose::ConstPtr msg)
  {
      ROS_INFO("[CV_NODE]: x: [%f] y:[%f] z: [%f]", msg->position.x, msg->position.y, msg->position.z);
      ROS_INFO("[CV_NODE]: qx: [%f] qy:[%f] qz: [%f] qw:[%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
      goal_pose = msg;
      move();
  }

  void move()
  {
    namespace rvt = rviz_visual_tools;
    std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    // set goal pose
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = goal_pose->orientation.w;
    target_pose1.orientation.x = goal_pose->orientation.x;
    target_pose1.orientation.y = goal_pose->orientation.y;
    target_pose1.orientation.z = goal_pose->orientation.z;
    target_pose1.position.x = goal_pose->position.x;
    target_pose1.position.y = goal_pose->position.y;
    target_pose1.position.z = goal_pose->position.z;

    // target_pose1.orientation.w = 1.0;
    // target_pose1.position.x = 0.28;
    // target_pose1.position.y = -0.2;
    // target_pose1.position.z = 0.5;

    move_group_interface.setPoseTarget(target_pose1);
    ROS_INFO("target set");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    ROS_INFO("target set1");
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("target set2");
    ROS_INFO("Plan %s", success ? "success" : "failure");

    // visualize plan
    visual_tools.publishAxisLabeled(target_pose1, "pose 1");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();

    bool success_execute = (move_group_interface.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Plan %s", success_execute ? "success" : "failure");
    ROS_INFO("Successfully executed!");
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  
  
  //setup visualizer
  MyMoveGroupInterface my_move;
  ros::waitForShutdown();
  return 0;
}