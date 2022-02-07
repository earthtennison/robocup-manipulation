#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

#include <moveit_grasps/grasp_generator.h>
#include "moveit_grasps/GraspPose.h"

namespace moveit_grasps
{
class GraspGeneratorDemo
{
private:

  ros::NodeHandle nh_;
  moveit_grasps::GraspGeneratorPtr grasp_generator_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  rviz_visual_tools::RvizVisualToolsPtr grasp_visuals_;
  moveit_grasps::GraspDataPtr grasp_data_;

  std::string ee_group_name_;

  geometry_msgs::Pose::ConstPtr objpose;
  ros::Publisher way_pub;
  const moveit::core::JointModelGroup* ee_jmg;

  public:
  // Constructor
  
  GraspGeneratorDemo() : nh_("~")
  {
    nh_.param("ee_group_name", ee_group_name_, std::string("hand"));
    ROS_INFO_STREAM_NAMED("demo", "End Effector: " << ee_group_name_);

    // ros::Publisher 
    way_pub = nh_.advertise<moveit_grasps::GraspPose>("/graspgen/waypoint", 2);

    grasp_data_.reset(new moveit_grasps::GraspData(nh_, ee_group_name_, visual_tools_->getRobotModel()));
    // const moveit::core::JointModelGroup* 
    ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup(ee_group_name_);
    
    
    loadscene();
    setIdealGrasp(ee_jmg);
    setscore();

    while(true)
    {
      ros::Subscriber obj_sub = nh_.subscribe("/obj_pose", 1000, Callback, this);
    }
  }

  void loadscene()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("world"));
    visual_tools_->setMarkerTopic("/rviz_visual_tools");
    visual_tools_->loadMarkerPub();
    visual_tools_->loadRobotStatePub("/display_robot_state");
    visual_tools_->loadTrajectoryPub("/display_planned_path");
    visual_tools_->loadSharedRobotState();
    visual_tools_->getSharedRobotState()->setToDefaultValues();
    visual_tools_->enableBatchPublishing();
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->hideRobot();
    visual_tools_->trigger();

    grasp_visuals_.reset(new rviz_visual_tools::RvizVisualTools("world"));
    grasp_visuals_->setMarkerTopic("/grasp_visuals");
    grasp_visuals_->loadMarkerPub();
    grasp_visuals_->enableBatchPublishing();
    grasp_visuals_->deleteAllMarkers();
    grasp_visuals_->trigger();
  }

  void setIdealGrasp(const moveit::core::JointModelGroup* ee_jmg)
  {
    // Set the ideal grasp
    Eigen::Isometry3d ideal_grasp_pose = Eigen::Isometry3d::Identity();
    ideal_grasp_pose.translation() = Eigen::Vector3d(0.0, 0, 0.5);
    grasp_generator_->setIdealTCPGraspPose(ideal_grasp_pose);
 
    std::vector<double> ideal_grasp_rpy = { 3.14, 0.0, 1.57 };
    grasp_generator_->setIdealTCPGraspPoseRPY(ideal_grasp_rpy);

    // Visualize the ideal grasp pose
    grasp_visuals_->publishAxisLabeled(grasp_generator_->ideal_grasp_pose_, "IDEAL_TCP_GRASP_POSE");
    visual_tools_->publishEEMarkers(grasp_generator_->ideal_grasp_pose_ * grasp_data_->tcp_to_eef_mount_, ee_jmg,
                                    grasp_data_->grasp_posture_.points[0].positions, rviz_visual_tools::BLUE);
    grasp_visuals_->publishAxisLabeled(grasp_generator_->ideal_grasp_pose_ * grasp_data_->tcp_to_eef_mount_,
                                       "IDEAL EEF MOUNT POSE");
    visual_tools_->trigger();
  }

  void setscore()
  {
    moveit_grasps::GraspScoreWeights grasp_score_weights;
    grasp_score_weights.orientation_x_score_weight_ = 2.0;
    grasp_score_weights.orientation_y_score_weight_ = 2.0;
    grasp_score_weights.orientation_z_score_weight_ = 2.0;
    grasp_score_weights.translation_x_score_weight_ = 1.0;
    grasp_score_weights.translation_y_score_weight_ = 1.0;
    grasp_score_weights.translation_z_score_weight_ = 1.0;

    grasp_score_weights.depth_score_weight_ = 2.0;
    grasp_score_weights.width_score_weight_ = 2.0;
    grasp_generator_->setGraspScoreWeights(grasp_score_weights);
  }

  void usingTestObject(geometry_msgs::Pose& object_pose,geometry_msgs::Pose::ConstPtr& got_object_pose)
  {
    geometry_msgs::Pose start_object_pose;

    // Position
    start_object_pose.position.x = got_object_pose -> position.x;
    start_object_pose.position.x = got_object_pose -> position.y;
    start_object_pose.position.x = got_object_pose -> position.z;

    // Orientation
    start_object_pose.orientation.w = got_object_pose -> orientation.w;
    start_object_pose.orientation.x = got_object_pose -> orientation.x;
    start_object_pose.orientation.y = got_object_pose -> orientation.y;
    start_object_pose.orientation.z = got_object_pose -> orientation.z;

    object_pose = start_object_pose;
  }

  void generateTestObject(geometry_msgs::Pose& object_pose)
  {
    geometry_msgs::Pose start_object_pose;

    // Position
    start_object_pose.position.x = 0.5;
    start_object_pose.position.y = 0.0;
    start_object_pose.position.z = 0.5;

    // Orientation
    double angle = M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_object_pose.orientation.x = quat.x();
    start_object_pose.orientation.y = quat.y();
    start_object_pose.orientation.z = quat.z();
    start_object_pose.orientation.w = quat.w();

    object_pose = start_object_pose;
  }

  void generateGrasp(const moveit::core::JointModelGroup* ee_jmg, ros::Publisher& way_pub,geometry_msgs::Pose object_pose)
  {
    std::vector<moveit_grasps::GraspCandidatePtr> possible_grasps;

    // Configure the desired types of grasps
    moveit_grasps::GraspCandidateConfig grasp_generator_config = moveit_grasps::GraspCandidateConfig();
    grasp_generator_config.disableAll();
    grasp_generator_config.enable_face_grasps_ = true;
    // grasp_generator_config.enable_edge_grasps_ = true;
    grasp_generator_config.generate_x_axis_grasps_ = true;
    grasp_generator_config.generate_y_axis_grasps_ = true;
    grasp_generator_config.generate_z_axis_grasps_ = true;


    double depth = 0.03;
    double width = 0.03;
    double height = 0.03;

    possible_grasps.clear();

    grasp_visuals_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::GREEN);
    grasp_visuals_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);
    grasp_visuals_->trigger();

    grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), depth, width, height, grasp_data_,
                                      possible_grasps, grasp_generator_config);

    if (possible_grasps.size() > 0)
    {
      visual_tools_->publishEEMarkers(possible_grasps.front()->grasp_.grasp_pose.pose, ee_jmg,
                                      grasp_data_->pre_grasp_posture_.points[0].positions, rviz_visual_tools::TRANSLUCENT_LIGHT,
                                      "demo_eef");
      visual_tools_->trigger();

      publish_waypoint(possible_grasps.front(),way_pub);

    }
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

  void Callback(const geometry_msgs::Pose::ConstPtr objmsg)
  {
    objpose = objmsg;

    geometry_msgs::Pose object_pose;
    usingTestObject(object_pose, objpose);
    generateGrasp(ee_jmg, way_pub, object_pose);
  }

};
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grasp_generator_demo");

  ROS_INFO_STREAM_NAMED("main", "GraspGenerator Demo");

  

  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit_grasps::GraspGeneratorDemo tester();
  while(ros::ok)
  {
    // ros::Subscriber obj_sub = nh_.subscribe("/obj_pose", 1000, tester().Callback);
  }

  ros::waitForShutdown();
}
