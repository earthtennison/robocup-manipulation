#include <ros/ros.h>

#include "darknet_ros_3d/Darknet3D.h"
#include "test_service/obj2pose_service.h"
#include "actionlib/server/action_server.h"
#include "test_service/obj2pose_actionAction.h"

bool requestCb(test_service::obj2pose_service::Request &req,
                test_service::obj2pose_service::Response &res){
  if (req.obj2pose_request == true){
    while (ros::ok())
    {
      darknet_ros_3d::Darknet3D darknet3d;
      darknet3d.update();
      // ros::spinOnce();
      res.obj2pose_respond = true;
      ROS_INFO("Finish Callback request");
      break;

    }
  }
  else{
    res.obj2pose_respond = false;
    ROS_ERROR("Something wrong Request is false");
  }
  

  return true;
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "darknet_3d");
  darknet_ros_3d::Darknet3D darknet3d;
  ros::Rate loop_rate(10);

  //---edit
  // ros::NodeHandle nh;
  // ros::ServiceServer server = nh.advertiseService("obj2pose_service",requestCb);
  // ros::spin();
  // ros::waitForShutdown();


  while (ros::ok())
  {
    darknet3d.update();

    ros::spinOnce();
    loop_rate.sleep();
  }

  
  return 0;
}


class Darknet3dAction
{
  protected:
    ros::NodeHandle nh_;
    actionlib::ActionServer<test_service::obj2pose_actionAction> as_;
    test_service::obj2pose_actionActionFeedback feedback_;
    test_service::obj2pose_actionActionResult result_;
    std::string action_name_;
  
  public:

    Darknet3dAction(std::string name):
    as_(nh_, name, boost::bind(&Darknet3dAction::ActionCB, this, _1), false),
    action_name_(name)
    {
      as_.start();
    }

    ~Darknet3dAction(void){}

    void ActionCB(const test_service::obj2pose_actionActionGoalConstPtr &goal)
    {
      ros::Rate loop_rate(10);
      bool successs = true;

      



    }
}
