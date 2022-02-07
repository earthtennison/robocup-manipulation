#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseArray.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char* argv[])
{
    geometry_msgs::Pose object_pose;

    object_pose.position.x = 0.5;
    object_pose.position.y = 0.0;
    object_pose.position.z = 0.2;

    double angle = M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    object_pose.orientation.x = quat.x();
    object_pose.orientation.y = quat.y();
    object_pose.orientation.z = quat.z();
    object_pose.orientation.w = quat.w();

    //------------------------------------------------------------------------------------------------
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(0);
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");

    ros::Publisher obj_pub = n.advertise<geometry_msgs::Pose>("/obj_pose", 2);

    
    spinner.start();
    while(ros::ok){
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to publish obj");
        obj_pub.publish(object_pose);
    }
}
