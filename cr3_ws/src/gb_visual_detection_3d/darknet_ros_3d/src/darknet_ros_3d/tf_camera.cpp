#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

   tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;

  
  transformStamped.header.frame_id = "dummy_link";
  transformStamped.child_frame_id = "camera_link";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = -0.35;
  transformStamped.transform.translation.z = 0.04;
  tf2::Quaternion q;
        q.setRPY(0, 0, -1.57);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  ros::Rate rate(100);
  while (node.ok()){
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    rate.sleep();

  }

};