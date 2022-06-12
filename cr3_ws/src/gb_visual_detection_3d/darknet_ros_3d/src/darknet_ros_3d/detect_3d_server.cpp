/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2019, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */
/* Author: Fernando González fergonzaramos@yahoo.es  */

#include "darknet_ros_3d/detect_3d_server.h"

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <limits>
#include <algorithm>


namespace darknet_ros_3d
{

Darknet3D::Darknet3D():
  nh_("~")
{
  initParams();
  darknet3d_server = nh_.advertiseService("dn3d_service", &Darknet3D::detect_3d,this);
  // last_detection_ts_ = ros::Time::now() - ros::Duration(60.0);
  
}

void
Darknet3D::initParams()
{
  working_frame_ = "camera_link";
  mininum_detection_thereshold_ = 0.2f;
  minimum_probability_ = 0.2f;


  nh_.param("working_frame", working_frame_, working_frame_);
  nh_.param("mininum_detection_thereshold", mininum_detection_thereshold_, mininum_detection_thereshold_);
  nh_.param("minimum_probability", minimum_probability_, minimum_probability_);

}

bool Darknet3D::detect_3d(gb_visual_detection_3d_msgs::Detect3d::Request &req, gb_visual_detection_3d_msgs::Detect3d::Response &res)
{
  
  sensor_msgs::PointCloud2 local_pointcloud;

  try
  {
    pcl_ros::transformPointCloud(working_frame_, req.point_cloud, local_pointcloud, tfListener_);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
    return true;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(local_pointcloud, *pcrgb);

  calculate_boxes(local_pointcloud, pcrgb, req.bounding_box, &res.bounding_box_3d);
  return true;

}


void
Darknet3D::calculate_boxes(const sensor_msgs::PointCloud2& cloud_pc2,
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_pcl,
    darknet_ros_msgs::BoundingBox bbx,
    gb_visual_detection_3d_msgs::BoundingBox3d* bounding_box_3d)
{
  bounding_box_3d->header.stamp = cloud_pc2.header.stamp;
  bounding_box_3d->header.frame_id = working_frame_;

  int center_x, center_y;

  center_x = (bbx.xmax + bbx.xmin) / 2;
  center_y = (bbx.ymax + bbx.ymin) / 2;

  int pcl_index = (center_y* cloud_pc2.width) + center_x;
  pcl::PointXYZRGB center_point =  cloud_pcl->at(pcl_index);

  if (std::isnan(center_point.x))
    return;

  float maxx, minx, maxy, miny, maxz, minz;

  maxx = maxy = maxz =  -std::numeric_limits<float>::max();
  minx = miny = minz =  std::numeric_limits<float>::max();

  for (int i = bbx.xmin; i < bbx.xmax; i++)
    for (int j = bbx.ymin; j < bbx.ymax; j++)
    {
      pcl_index = (j* cloud_pc2.width) + i;
      pcl::PointXYZRGB point =  cloud_pcl->at(pcl_index);

      if (std::isnan(point.x))
        continue;

      if (fabs(point.x - center_point.x) > mininum_detection_thereshold_)
        continue;

      maxx = std::max(point.x, maxx);
      maxy = std::max(point.y, maxy);
      maxz = std::max(point.z, maxz);
      minx = std::min(point.x, minx);
      miny = std::min(point.y, miny);
      minz = std::min(point.z, minz);
    }


  gb_visual_detection_3d_msgs::BoundingBox3d bbx_msg;
  bbx_msg.Class = bbx.Class;
  bbx_msg.probability = bbx.probability;
  bbx_msg.xmin = minx;
  bbx_msg.xmax = maxx;
  bbx_msg.ymin = miny;
  bbx_msg.ymax = maxy;
  bbx_msg.zmin = minz;
  bbx_msg.zmax = maxz;

  // boxes->bounding_boxes.push_back(bbx_msg);
  bounding_box_3d = &bbx_msg;
}

  
};  // namespace darknet_ros_3d

int main(int argc, char **argv)
{
  darknet_ros_3d::Darknet3D darknet3d;
  ros::Rate loop_rate(10);

  ros::init(argc, argv, "add_two_ints_server");
  darknet3d.update();
  ros::spinOnce();
  loop_rate.sleep();

  return 0;
}
