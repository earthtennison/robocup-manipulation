#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_controller");
    ros::NodeHandle node_handle;
    

    ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::Pose>("Position", 2);
    ros::AsyncSpinner spinner(0);
    
    if (ros::ok())
    {
        geometry_msgs::Pose nba;
        double roll,pitch,yaw;

        std::cout<<"roll: " ;  std::cin >> roll;
        std::cout<<"pitch: " ; std::cin >> pitch;
        std::cout<<"yaw: " ; std::cin >> yaw;

        tf2::Quaternion orientation;
        orientation.setRPY(roll, pitch, yaw);
        nba.orientation = tf2::toMsg(orientation);
        std::cout<< "x: "; std::cin >> nba.position.x;
        std::cout<< "y: "; std::cin >> nba.position.y;
        std::cout<< "z: "; std::cin >> nba.position.z;
    
        ROS_INFO_STREAM("------Position Set------\n" << nba);

        pose_pub.publish(nba);
        spinner.start();
    }

    return 0;

}