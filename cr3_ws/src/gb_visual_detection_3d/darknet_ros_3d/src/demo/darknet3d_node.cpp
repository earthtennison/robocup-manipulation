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

#include <ros/ros.h>

#include "darknet_ros_3d/Darknet3D.h"
#include "test_service/obj2pose_service.h"

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
