#include "darknet_ros_3d/detect_3d_server.h"
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include<sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <limits>
#include <algorithm>
namespace darknet_ros_3d
{
  Darknet3D::Darknet3D() : nh_("~")
  {
    initParams();
    darknet3d_server = nh_.advertiseService("dn3d_service", &Darknet3D::detect_3d, this);
  }
  void
  Darknet3D::initParams()
  {
    working_frame_ = "object_frame";
    mininum_detection_thereshold_ = 0.2f;
    // min_depth_thereshold = 0.1;
    // nh_.param("working_frame", working_frame_, working_frame_);
    // nh_.param("mininum_detection_thereshold", mininum_detection_thereshold_, mininum_detection_thereshold_);
  }
  bool Darknet3D::detect_3d(gb_visual_detection_3d_msgs::Detect3d::Request &req, gb_visual_detection_3d_msgs::Detect3d::Response &res)
  {
    ROS_INFO("Reciving Service");
    sensor_msgs::PointCloud2 local_pointcloud;
    try
    {
      pcl_ros::transformPointCloud(working_frame_, req.point_cloud, local_pointcloud, tfListener_);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
      return true;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(local_pointcloud, *pcrgb);
    gb_visual_detection_3d_msgs::BoundingBox3d bounding_box_3d;
    bool success = calculate_boxes(local_pointcloud, pcrgb, req.bounding_box, bounding_box_3d);
    res.success = success;
    res.bounding_box_3d = bounding_box_3d;
    return true;
  }
  bool Darknet3D::calculate_boxes(const sensor_msgs::PointCloud2 &cloud_pc2,
                                  const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_pcl,
                                  darknet_ros_msgs::BoundingBox bbx,
                                  gb_visual_detection_3d_msgs::BoundingBox3d &bounding_box_3d)
  {
    // init_z = false;
    // ROS_INFO_STREAM(init_z);
    int center_x, center_y;
    center_x = (bbx.xmax + bbx.xmin) / 2;
    center_y = (bbx.ymax + bbx.ymin) / 2;
    int pcl_index = (center_y * cloud_pc2.width) + center_x;
    pcl::PointXYZRGB center_point = cloud_pcl->at(pcl_index);
    // if (std::isnan(center_point.x))
    //   return false;
    float maxx, minx, maxy, miny, maxz, minz;
    maxx = maxy = maxz = -std::numeric_limits<float>::max();
    minx = miny = minz = std::numeric_limits<float>::max();


    for (int i = bbx.xmin; i < bbx.xmax; i++)
      for (int j = bbx.ymin; j < bbx.ymax; j++)
      {
        pcl_index = (j * cloud_pc2.width) + i;
        pcl::PointXYZRGB point = cloud_pcl->at(pcl_index);
        if (std::isnan(point.x))
          continue;
        minz = std::min(point.z, minz);
      }


    for (int i = bbx.xmin; i < bbx.xmax; i++)
      for (int j = bbx.ymin; j < bbx.ymax; j++)
      {
        pcl_index = (j * cloud_pc2.width) + i;
        pcl::PointXYZRGB point = cloud_pcl->at(pcl_index);
        if (std::isnan(point.x))
          continue;
        // if (fabs(point.x - center_point.x) > mininum_detection_thereshold_)
        //   continue;
        // if (init_z = true){
        //   if ((minz-point.z > min_depth_thereshold) || (point.z - maxz > min_depth_thereshold)){
        //     continue;
        //   }
        // }
        
        maxx = std::max(point.x, maxx);
        maxy = std::max(point.y, maxy);
        maxz = std::max(point.z, maxz);
        minx = std::min(point.x, minx);
        miny = std::min(point.y, miny);
        minz = std::min(point.z, minz);
        
      //   if (init_z = false){
      //   if ((maxz > -std::numeric_limits<float>::max())){
      //     ROS_INFO_STREAM(init_z);
      //     init_z = true;
      //     ROS_INFO_STREAM(init_z);
      //   }}
      }
    bounding_box_3d.Class = bbx.Class;
    bounding_box_3d.probability = bbx.probability;
    bounding_box_3d.xmin = minx;
    bounding_box_3d.xmax = maxx;
    bounding_box_3d.ymin = miny;
    bounding_box_3d.ymax = maxy;
    bounding_box_3d.zmin = minz;
    bounding_box_3d.zmax = maxz;
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "GGEZ");
  darknet_ros_3d::Darknet3D darknet3d;
  ros::Rate loop_rate(10);
  darknet3d.update();
  ros::spin();
  return 0;
}