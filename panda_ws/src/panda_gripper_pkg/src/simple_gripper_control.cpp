// #include <ros/ros.h>
// #include <string>

// #define JOINT_CONTROL_TOPIC "/panda_arm/gripper_control"

// int main(int argc, char **argv){
//     ros::init(argc, argv, "simple_gripper_control.cpp");
//     ros::NodeHandle nh;

//     std::string jointStatePublishTopic(JOINT_CONTROL_TOPIC);
//     ros::Publisher pub = nh.advertise<sensor_msgs::JointState>(jointStatePublishTopic);    
//     return 0;
// }