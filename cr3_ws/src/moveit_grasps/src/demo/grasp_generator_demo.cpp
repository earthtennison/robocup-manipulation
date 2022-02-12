#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>


#include <moveit_grasps/grasp_generator.h>
#include "moveit_grasps/GraspPose.h"

namespace moveit_grasps
{
class GraspGeneratorDemo
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Which arm should be used
  std::string ee_group_name_;

  
  
  ros::Publisher way_pub;
  ros::Publisher pose_pub;
  ros::Subscriber obj_sub;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
    // Grasp generator
  moveit_grasps::GraspGeneratorPtr grasp_generator_;

  // Tool for publishing stuff to rviz
  // moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  rviz_visual_tools::RvizVisualToolsPtr grasp_visuals_;

  // Robot-specific data for generating grasps
  moveit_grasps::GraspDataPtr grasp_data_;

  geometry_msgs::Pose::ConstPtr objpose;
  

public:
  // Constructor
  GraspGeneratorDemo(int num_tests) : nh_("~")
  {
    nh_.param("ee_group_name", ee_group_name_, std::string("hand"));

    //<EDIT-----------------------------------------------------------------------------------------

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_link"));
    visual_tools_->setMarkerTopic("/rviz_visual_tools");
    visual_tools_->loadMarkerPub();
    visual_tools_->loadRobotStatePub("/display_robot_state");
    visual_tools_->loadTrajectoryPub("/display_planned_path");
    visual_tools_->loadSharedRobotState();
    visual_tools_->getSharedRobotState()->setToDefaultValues();
    visual_tools_->enableBatchPublishing();
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    // visual_tools_->hideRobot();
    visual_tools_->trigger();

    // TODO(davetcoleman): do we need two VisualTools? ideally consolidate
    grasp_visuals_.reset(new rviz_visual_tools::RvizVisualTools("base_link"));
    grasp_visuals_->setMarkerTopic("/grasp_visuals");
    grasp_visuals_->loadMarkerPub();
    grasp_visuals_->enableBatchPublishing();
    grasp_visuals_->deleteAllMarkers();
    grasp_visuals_->trigger();

    way_pub = nh_.advertise<geometry_msgs::Pose>("/graspgen/waypoint", 2);
    // way_pub = nh_.advertise<moveit_grasps::GraspPose>("/graspgen/waypoint", 2);
    obj_sub = nh_.subscribe("/obj_pose", 1000, &GraspGeneratorDemo::objCallback, this);

    
  }

  void objCallback(const geometry_msgs::Pose::ConstPtr pose_msg)
  {


    ROS_INFO_STREAM("got obj pose");
    ROS_INFO_STREAM(pose_msg);

    objpose = pose_msg;

    ROS_INFO_STREAM_NAMED("demo", "End Effector: " << ee_group_name_);


    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    grasp_data_.reset(new moveit_grasps::GraspData(nh_, ee_group_name_, visual_tools_->getRobotModel()));

    const moveit::core::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup(ee_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    grasp_generator_.reset(new moveit_grasps::GraspGenerator(visual_tools_, true));
    grasp_generator_->setVerbose(true);

    // ---------------------------------------------------------------------------------------------
    // Set the ideal grasp pose to be centered and 0.5m above (for visualization) with an orientation of roll = 3.14

    // Set the translation
    Eigen::Isometry3d ideal_grasp_pose = Eigen::Isometry3d::Identity();
    ideal_grasp_pose.translation() = Eigen::Vector3d(0.0, 0, 0.5);
    grasp_generator_->setIdealTCPGraspPose(ideal_grasp_pose);

    // Set the ideal grasp orientation
    std::vector<double> ideal_grasp_rpy = { 3.14, 0.0, 1.57 };
    grasp_generator_->setIdealTCPGraspPoseRPY(ideal_grasp_rpy);

    // ---------------------------------------------------------------------------------------------
    // Visualize the ideal grasp pose
    grasp_visuals_->publishAxisLabeled(grasp_generator_->ideal_grasp_pose_, "IDEAL_TCP_GRASP_POSE");
    visual_tools_->publishEEMarkers(grasp_generator_->ideal_grasp_pose_ * grasp_data_->tcp_to_eef_mount_, ee_jmg,
                                    grasp_data_->grasp_posture_.points[0].positions, rviz_visual_tools::BLUE);
    grasp_visuals_->publishAxisLabeled(grasp_generator_->ideal_grasp_pose_ * grasp_data_->tcp_to_eef_mount_,
                                       "IDEAL EEF MOUNT POSE");
    visual_tools_->trigger();

    // ---------------------------------------------------------------------------------------------
    // We also set custom grasp score weights
    moveit_grasps::GraspScoreWeights grasp_score_weights;
    grasp_score_weights.orientation_x_score_weight_ = 2.0;
    grasp_score_weights.orientation_y_score_weight_ = 2.0;
    grasp_score_weights.orientation_z_score_weight_ = 2.0;
    grasp_score_weights.translation_x_score_weight_ = 1.0;
    grasp_score_weights.translation_y_score_weight_ = 1.0;
    grasp_score_weights.translation_z_score_weight_ = 1.0;

    // Finger gripper specific weights. (Note that we do not need to set the suction gripper specific weights for our
    // finger gripper)
    grasp_score_weights.depth_score_weight_ = 2.0;
    grasp_score_weights.width_score_weight_ = 2.0;
    grasp_generator_->setGraspScoreWeights(grasp_score_weights);

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    geometry_msgs::Pose object_pose;
    std::vector<moveit_grasps::GraspCandidatePtr> possible_grasps;

    // Configure the desired types of grasps
    moveit_grasps::GraspCandidateConfig grasp_generator_config = moveit_grasps::GraspCandidateConfig();
    grasp_generator_config.disableAll();
    grasp_generator_config.enable_face_grasps_ = true;
    // grasp_generator_config.enable_edge_grasps_ = true;
    grasp_generator_config.generate_x_axis_grasps_ = true;
    grasp_generator_config.generate_y_axis_grasps_ = true;
    grasp_generator_config.generate_z_axis_grasps_ = true;

    // Loop
    while (ros::ok())
    {
      ROS_INFO_STREAM_NAMED("demo", "Adding random posed object ");

      object_pose.orientation.w = objpose->orientation.w;
      object_pose.orientation.x = objpose->orientation.x;
      object_pose.orientation.y = objpose->orientation.y;
      object_pose.orientation.z = objpose->orientation.z;
      object_pose.position.x = objpose->position.x;
      object_pose.position.y = objpose->position.y;
      object_pose.position.z = objpose->position.z;
      
      possible_grasps.clear();

      // Generate set of grasps for one object
      double depth = 0.03;
      double width = 0.03;
      double height = 0.03;

      grasp_visuals_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::GREEN);
      grasp_visuals_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);
      grasp_visuals_->trigger();

      
      
      
      grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), depth, width, height, grasp_data_,
                                       possible_grasps, grasp_generator_config);

      
      ROS_INFO("Finished generating grasp,begining planing");
      

      if (possible_grasps.size() > 0)
      {
        visual_tools_->publishEEMarkers(possible_grasps.front()->grasp_.grasp_pose.pose, ee_jmg,
                                        grasp_data_->pre_grasp_posture_.points[0].positions, rviz_visual_tools::TRANSLUCENT_LIGHT,
                                        "demo_eef");
        
        way_pub.publish(possible_grasps.front()->grasp_.grasp_pose.pose);

        // publish_waypoint(possible_grasps.front(),way_pub);
        // excute_catesian(possible_grasps.front());

        visual_tools_->trigger();
      }
      break;
    }
  }

  void excute_catesian(moveit_grasps::GraspCandidatePtr& valid_grasp_candidate)
  {
    geometry_msgs::Pose pose; 
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    moveit::planning_interface::MoveGroupInterface move_group("arm");
    moveit_msgs::RobotTrajectory trajectory;
    EigenSTL::vector_Isometry3d grasp_waypoints;
    std::vector<geometry_msgs::Pose> Cwaypoint;

    GraspGenerator::getGraspWaypoints(valid_grasp_candidate, grasp_waypoints);

    tf::poseEigenToMsg(grasp_waypoints[0],pose);
    visual_tools_->publishAxisLabeled(grasp_waypoints[0], "state_0", rviz_visual_tools::SMALL);
    move_group.setPoseTarget(pose);
    move_group.move();


    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    for (int state = 1; state < 4; state++)
    {
      tf::poseEigenToMsg(grasp_waypoints[state],pose);
      visual_tools_->publishAxisLabeled(grasp_waypoints[state], "state_" + std::to_string(state), rviz_visual_tools::SMALL);
      Cwaypoint.push_back(pose);
    }

    double fraction = move_group.computeCartesianPath(Cwaypoint, eef_step, jump_threshold, trajectory);

    ROS_INFO_NAMED("Test", "Visualizing Cartesian path (%.2f%% acheived)", fraction * 100.0);

    grasp_plan.trajectory_ = trajectory;
    move_group.execute(grasp_plan);
  }

  void publish_waypoint(moveit_grasps::GraspCandidatePtr& valid_grasp_candidate, ros::Publisher& pose_publisher)
  {//---Get grasp candidate and publish vector with pre-grasp, grasp, lift and retreat poses--//

    geometry_msgs::Pose pose; 
    EigenSTL::vector_Isometry3d grasp_waypoints;
    moveit_grasps::GraspPose waypoints_pose;

    GraspGenerator::getGraspWaypoints(valid_grasp_candidate, grasp_waypoints);

    waypoints_pose.grasp_poses.clear();
    for (int state = 0; state < 4; state++)
    {
      tf::poseEigenToMsg(grasp_waypoints[state],pose);
      visual_tools_->publishAxisLabeled(grasp_waypoints[state], "state_" + std::to_string(state), rviz_visual_tools::SMALL);
      waypoints_pose.grasp_poses.push_back(pose);
      
    }

    pose_publisher.publish(waypoints_pose);
  }

};  // end of class

}  // namespace

int main(int argc, char* argv[])
{
  int num_tests = 1;
  ros::init(argc, argv, "grasp_generator_demo");

  ROS_INFO_STREAM_NAMED("main", "GraspGenerator Demo");

  ros::AsyncSpinner spinner(0);
  spinner.start();

  // Run Demos
  moveit_grasps::GraspGeneratorDemo tester(num_tests);

  ros::waitForShutdown();
  return 0;
}