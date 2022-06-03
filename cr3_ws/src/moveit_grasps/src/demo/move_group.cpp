#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void inpose(const geometry_msgs::Pose& pose)
{

    ROS_INFO_STREAM("Set");
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface arm_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;

    // arm_group.setPlanningTime(2);

    arm_group.setPoseTarget(pose);
    
    if (arm_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS){
      ROS_INFO("Planning sucessful");
      arm_group.asyncExecute(my_plan);
    }
    else{
      ROS_INFO("Planning Failed");
    }
    
    
    
    ROS_INFO_STREAM("Move");

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_group_interface");    
    ros::NodeHandle node_handle;
    geometry_msgs::Pose Dpose;
    

    ros::Subscriber pose_sub = node_handle.subscribe("/moveit_grasps_demo/Position",2,inpose);
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::waitForShutdown();

}