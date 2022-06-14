#include "ros/ros.h"
#include "try_ros_service_and_client/kan_pick.h"

class Base
{
public:
  virtual bool add(try_ros_service_and_client::kan_pick::Request &req,
		   try_ros_service_and_client::kan_pick::Response &res)
  {
    double px, py, pz, ox, oy, oz, ow;
    px = req.geo_req.position.x;
    py = req.geo_req.position.y;
    pz = req.geo_req.position.z;
    ox = req.geo_req.orientation.x;
    oy = req.geo_req.orientation.y;
    oz = req.geo_req.orientation.z;
    ow = req.geo_req.orientation.w;
    ROS_INFO("request: px=%lf, py=%lf, pz=%lf, ox=%lf, oy=%lf, oz=%lf, ow=%lf", (double)px, (double)py, (double)pz, (double)ox, (double)oy, (double)oz, (double)ow);
    ROS_INFO(" send back response: [%B]", (bool)res.success_grasp);

    return true;
  }

  virtual int op(int a, int b);
  {


  }
};

class AddTwo: public Base
{
public:
  virtual int op(double px, double py, double pz, double ox, double oy, double oz, double ow)
  {
    ROS_INFO("calling derived op()");
    return 4;

  };

  
};

int main(int argc, char **argv){
  ros::init(argc, argv, "pose_receiver");
  ros::NodeHandle nh;

  AddTwo a;
  ros::Service serv = nh.advertiseService("receiver", &AddTwo::add, (Base*) &a);
  ros::spin();

  return 0;

}
