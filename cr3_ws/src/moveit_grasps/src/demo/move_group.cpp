#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void inpose(const geometry_msgs::Pose& pose)
{

    ROS_INFO_STREAM("Set");
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface arm_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveit_msgs::RobotTrajectory trajectory;
   

    arm_group.setPoseTarget(pose);
    bool success = (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success == true){
      ROS_INFO("Planning sucessful");
      arm_group.move();
    }

    
    
    ROS_INFO_STREAM("Move");

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface");    
    ros::NodeHandle node_handle;
    geometry_msgs::Pose Dpose;

    ros::Subscriber pose_sub = node_handle.subscribe("/graspgen/waypoint",2,inpose);
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();

}