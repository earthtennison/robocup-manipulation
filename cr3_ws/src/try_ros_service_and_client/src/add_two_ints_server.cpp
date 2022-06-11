#include "ros/ros.h"
#include "try_ros_service_and_client/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request &req,
	 beginner_tutorials::AddTwoInts::Response &res){
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ROS_SERVICE_NODE_2D_AND_3D");
  ros::NodeHandle n;
  ros::Serviceserver service = node2d3d.advertiseService("add_two_ints", 1000);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
