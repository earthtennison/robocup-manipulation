#include "ros/ros.h"
#include "cr3_moveit_control/kan_place.h"

bool kan_place_server(cr3_moveit_control::kan_place::Request &req,
		      cr3_moveit_control::kan_place::Response &res){
  double pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w;
  pos_x = req.geo_req.position.x;
  pos_y = req.geo_req.position.y;
  pos_z = req.geo_req.position.z;
  ori_x = req.geo_req.orientation.x;
  ori_y = req.geo_req.orientation.y;
  ori_z = req.geo_req.orientation.z;
  ori_w = req.geo_req.orientation.w;

  res.success_place = true;
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "place_success_server");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("place_success", kan_place_server);

  ros::waitForShutdown();
  return 0;
}
