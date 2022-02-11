#include "ros/ros.h"
#include <geometry_msgs/Pose.h>

class DoMove {
  private:
    ros::Publisher pub;
    ros::NodeHandle nh;
    geometry_msgs::Pose my_pose;
  public:
    DoMove(){
      pub = nh.advertise<geometry_msgs::Pose>("Position",100);
      publish_pose();

    }
    void publish_pose(){
      my_pose.position.x = 1;
      my_pose.position.y = 1;
      my_pose.position.z = 1;
      my_pose.orientation.x = 0;
      my_pose.orientation.y = 0;
      my_pose.orientation.z = 0;
      my_pose.orientation.w = 1;

      pub.publish(my_pose);

    }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "do_move");
  DoMove();

  while(ros::ok()){
    ros::spinOnce();
  }



}