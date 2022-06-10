#include "ros/ros.h"
#include "test_service/detectobj.h"
#include "sensor_msgs/PointCloud2.h"

class fakesmach_client
{
private:
  sensor_msgs::PointCloud2::ConstPtr *cap_pcl;
  ros::NodeHandle nh_;
  ros::Subscriber obj_sub;
  ros::ServiceClient client;
  test_service::detectobj srv;

public:
  fakesmach_client(std::string obj_name)
  {
    obj_sub = nh_.subscribe("/camera/depth_registered/points", 1, &fakesmach_client::callback,this);
    client = nh_.serviceClient<test_service::detectobj>("obj_detect",this);

    srv.request.obj_name = obj_name;

    
    try
    {
      client.call(srv);

      client.waitForExistence();
      ROS_INFO_STREAM(srv.response.complete);
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("error dadwdwdawdawdadwadwd");
    }
  }

  void callback(const sensor_msgs::PointCloud2::ConstPtr Pcl_msg){

    std::string input;
    std::cout << "press any to continue";
    std::cin >> input;

    try
    {
      *cap_pcl = Pcl_msg;
      ROS_INFO("capture point clound");
    }
    catch(const std::exception& e)
    {
      ROS_INFO_STREAM(e.what());
    }
    
  }

};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "smach_client");
  ros::NodeHandle nh;
  
  ros::AsyncSpinner spinner(0);
  spinner.start();

  fakesmach_client testclient("bottle");

  ros::waitForShutdown();
  return 0;

}
