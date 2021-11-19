/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Ridhwan Luthra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Ridhwan Luthra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ridhwan Luthra */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class SphereSegment
{
public:
  SphereSegment()
  {
    ros::NodeHandle nh;

    // ros::AsyncSpinner spinner(3);
    // spinner.start();
    // Initialize subscriber to the raw point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, &SphereSegment::cloudCB, this);
    // Spin
    // ros::spin();

    ros::AsyncSpinner spinner(0);
    spinner.start();
    // ros::waitForShutdown();
  }

  /** \brief Given the parameters of the cylinder add the cylinder to the planning scene.
      @param sphere_params - Pointer to the struct AddSphereParams. */
  void AddSphere()
  {
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // BEGIN_SUB_TUTORIAL add_cylinder
    //
    // Adding Cylinder to Planning Scene
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "panda_link0";
    collision_object.id = "sphere";

    // Define a cylinder which will be added to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(1);
    /* Setting height of cylinder. */
    primitive.dimensions[0] = sphere_params->radius;
    /* Setting radius of cylinder. */
    

    // Define a pose for the cylinder (specified relative to frame_id).
    geometry_msgs::Pose sphere_pose;
    /* Computing and setting quaternion from axis angle representation. */
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0); //(180, 0, -45)
    sphere_pose.orientation = tf2::toMsg(orientation);
    

    // Setting the position of cylinder.
    sphere_pose.position.x = sphere_params->center_pt[0];
    sphere_pose.position.y = sphere_params->center_pt[1];
    sphere_pose.position.z = sphere_params->center_pt[2];

    // Add cylinder as collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(sphere_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface.applyCollisionObject(collision_object);

    std::cout<<"segment finished";
    // END_SUB_TUTORIAL
  }

  /** \brief Given the pointcloud containing just the cylinder, compute its center point and its height and store in
     sphere_params.
      @param cloud - Pointcloud containing just the cylinder.
      @param sphere_params - Pointer to the struct AddSphereParams. */
  // void extractLocationHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  // {
  //   double max_angle_y = 0.0;
  //   double min_angle_y = std::numeric_limits<double>::infinity();

  //   double lowest_point[3];
  //   double highest_point[3];
  //   // BEGIN_SUB_TUTORIAL extract_location_height
  //   // Consider a point inside the point cloud and imagine that point is formed on a XY plane where the perpendicular
  //   // distance from the plane to the camera is Z. |br|
  //   // The perpendicular drawn from the camera to the plane hits at center of the XY plane. |br|
  //   // We have the x and y coordinate of the point which is formed on the XY plane. |br|
  //   // X is the horizontal axis and Y is the vertical axis. |br|
  //   // C is the center of the plane which is Z meter away from the center of camera and A is any point on the plane.
  //   // |br|
  //   // Now we know Z is the perpendicular distance from the point to the camera. |br|
  //   // If you need to find the  actual distance d from the point to the camera, you should calculate the hypotenuse-
  //   // |code_start| hypot(point.z, point.x);\ |code_end| |br|
  //   // angle the point made horizontally- |code_start| atan2(point.z,point.x);\ |code_end| |br|
  //   // angle the point made Vertically- |code_start| atan2(point.z, point.y);\ |code_end| |br|
  //   // Loop over the entire pointcloud.
  //   for (auto const point : cloud->points)
  //   {
  //     /* Find the coordinates of the highest point */
  //     if (atan2(point.z, point.y) < min_angle_y)
  //     {
  //       min_angle_y = atan2(point.z, point.y);
  //       lowest_point[0] = point.x;
  //       lowest_point[1] = point.y;
  //       lowest_point[2] = point.z;
  //     }
  //     /* Find the coordinates of the lowest point */
  //     else if (atan2(point.z, point.y) > max_angle_y)
  //     {
  //       max_angle_y = atan2(point.z, point.y);
  //       highest_point[0] = point.x;
  //       highest_point[1] = point.y;
  //       highest_point[2] = point.z;
  //     }
  //   }
  //   /* Store the center point of cylinder */
  //   sphere_params->center_pt[0] = (highest_point[0] + lowest_point[0]) / 2;
  //   sphere_params->center_pt[1] = (highest_point[1] + lowest_point[1]) / 2;
  //   sphere_params->center_pt[2] = (highest_point[2] + lowest_point[2]) / 2;
  //   /* Store the height of cylinder */
    
  //   // END_SUB_TUTORIAL
  // }

  /** \brief Given a pointcloud extract the ROI defined by the user.
      @param cloud - Pointcloud whose ROI needs to be extracted. */
  void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
  {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    // min and max values in z axis to keep
    pass.setFilterLimits(0.3, 1.1);
    pass.filter(*cloud);
  }

  /** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
      @param cloud - Pointcloud.
      @param cloud_normals - The point normals once computer will be stored in this. */
  void computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
  }

  /** \brief Given the point normals and point indices, extract the normals for the indices.
      @param cloud_normals - Point normals.
      @param inliers_plane - Indices whose normals need to be extracted. */
  void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals);
  }

  /** \brief Given the pointcloud and indices of the plane, remove the plannar region from the pointcloud.
      @param cloud - Pointcloud.
      @param inliers_plane - Indices representing the plane. */
  void removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers_plane)
  {
    // create a SAC segmenter without using normals
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    /* run at max 1000 iterations before giving up */
    segmentor.setMaxIterations(1000);
    /* tolerance for variation from model */
    segmentor.setDistanceThreshold(0.01);
    segmentor.setInputCloud(cloud);
    /* Create the segmentation object for the planar model and set all the parameters */
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    segmentor.segment(*inliers_plane, *coefficients_plane);
    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    /* Remove the planar inliers, extract the rest */
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);
  }

  /** \brief Given the pointcloud, pointer to pcl::ModelCoefficients and point normals extract the cylinder from the
     pointcloud and store the cylinder parameters in coefficients_sphere.
      @param cloud - Pointcloud whose plane is removed.
      @param coefficients_sphere - Cylinder parameters used to define an infinite cylinder will be stored here.
      @param cloud_normals - Point normals corresponding to the plane on which cylinder is kept */
  void extractSphere(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_sphere,
                       pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    // Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
    pcl::PointIndices::Ptr inliers_sphere(new pcl::PointIndices);
    segmentor.setOptimizeCoefficients(false);
    segmentor.setModelType(pcl::SACMODEL_SPHERE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    // Set the normal angular distance weight
    segmentor.setNormalDistanceWeight(0.1);
    // run at max 1000 iterations before giving up
    segmentor.setMaxIterations(100000);
    // tolerance for variation from model
    segmentor.setDistanceThreshold(0.05);
    // min max values of radius in meters to consider
    segmentor.setRadiusLimits(0, 1);
    segmentor.setInputCloud(cloud);
    segmentor.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    segmentor.segment(*inliers_sphere, *coefficients_sphere);

    // Extract the cylinder inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_sphere);
    extract.setNegative(false);
    extract.filter(*cloud);
  }

  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    // BEGIN_SUB_TUTORIAL callback
    //
    // Perception Related
    // ^^^^^^^^^^^^^^^^^^
    // First, convert from sensor_msgs to pcl::PointXYZRGB which is needed for most of the processing.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);
    // Using passthough filter to get region of interest. A passthrough filter just eliminates the point cloud values
    // which do not lie in the user specified range.
    passThroughFilter(cloud);
    // Declare normals and call function to compute point normals.
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    computeNormals(cloud, cloud_normals);
    // inliers_plane will hold the indices of the point cloud that correspond to a plane.
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    // Detect and eliminate the plane on which the cylinder is kept to ease the process of finding the cylinder.
    removePlaneSurface(cloud, inliers_plane);
    // We had calculated the point normals in a previous call to computeNormals,
    // now we will be extracting the normals that correspond to the plane on which cylinder lies.
    // It will be used to extract the cylinder.
    extractNormals(cloud_normals, inliers_plane);
    // ModelCoefficients will hold the parameters using which we can define a cylinder of infinite length.
    // It has a public attribute |code_start| values\ |code_end| of type |code_start| std::vector< float >\ |code_end|\
    // .
    // |br|
    // |code_start| Values[0-2]\ |code_end| hold a point on the center line of the cylinder. |br|
    // |code_start| Values[3-5]\ |code_end| hold direction vector of the z-axis. |br|
    // |code_start| Values[6]\ |code_end| is the radius of the cylinder.
    pcl::ModelCoefficients::Ptr coefficients_sphere(new pcl::ModelCoefficients);
    /* Extract the cylinder using SACSegmentation. */
    extractSphere(cloud, coefficients_sphere, cloud_normals);
    // END_SUB_TUTORIAL
    if (cloud->points.empty())
    {
      ROS_ERROR_STREAM_NAMED("sphere_segment", "Can't find the spherical component.");
      return;
    }
    if (points_not_found)
    {
      // BEGIN_TUTORIAL
      // CALL_SUB_TUTORIAL callback
      //
      // Storing Relevant Cylinder Values
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // The information that we have in |code_start| coefficients_sphere\ |code_end| is not enough to define our
      // cylinder.
      // It does not have the actual location of the cylinder nor the actual height. |br|
      // We define a struct to hold the parameters that are actually needed for defining a collision object completely.
      // |br|
      // CALL_SUB_TUTORIAL param_struct
      /* Store the radius of the cylinder. */
      sphere_params->center_pt[0] = coefficients_sphere->values[0];
      sphere_params->center_pt[1] = coefficients_sphere->values[1];
      sphere_params->center_pt[2] = coefficients_sphere->values[2];
      sphere_params->radius = coefficients_sphere->values[3];
      /* Store direction vector of z-axis of cylinder. */
      
      //
      // Extracting Location and Height
      // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // Compute the center point of the cylinder using standard geometry
      
      // CALL_SUB_TUTORIAL extract_location_height
      // Use the parameters extracted to add the cylinder to the planning scene as a collision object.
      AddSphere();
      // CALL_SUB_TUTORIAL add_cylinder
      // END_TUTORIAL
      points_not_found = false;
    }
  }

private:
  // BEGIN_SUB_TUTORIAL param_struct
  // There are 4 fields and a total of 7 parameters used to define this.
  struct AddSphereParams
  {
    /* Radius of the cylinder. */
    double radius;
    /* Direction vector towards the z-axis of the cylinder. */
    /* Center point of the cylinder. */
    double center_pt[3];
    /* Height of the cylinder. */
    // double height;
  };
  // Declare a variable of type AddSphereParams and store relevant values from ModelCoefficients.
  AddSphereParams* sphere_params;
  // END_SUB_TUTORIAL

  bool points_not_found = true;
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "sphere_segment");
  // Start the segmentor
  SphereSegment();
}