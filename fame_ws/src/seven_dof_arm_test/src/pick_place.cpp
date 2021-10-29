#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.0;
  posture.points[0].positions[1] = 0.0;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& current_scene)
{
  //initiate list of collision object
  std::vector<moveit_msgs::CollisionObject> collision_obj;
  collision_obj.resize(3);

  //define table1
  collision_obj[0].id = "table1";
  collision_obj[0].header.frame_id = "panda_link0";
  collision_obj[0].primitives.resize(1);
  collision_obj[0].primitives[0].type = collision_obj[0].primitives[0].BOX;
  collision_obj[0].primitives[0].dimensions.resize(3);
  collision_obj[0].primitives[0].dimensions[0] = 0.2;
  collision_obj[0].primitives[0].dimensions[1] = 0.4;
  collision_obj[0].primitives[0].dimensions[2] = 0.4;
  collision_obj[0].primitive_poses.resize(1);
  collision_obj[0].primitive_poses[0].position.x = 0.5;
  collision_obj[0].primitive_poses[0].position.y = 0;
  collision_obj[0].primitive_poses[0].position.z = 0.2;
  collision_obj[0].operation = collision_obj[0].ADD;

  //define table2
  collision_obj[1].id = "table2";
  collision_obj[1].header.frame_id = "panda_link0";
  collision_obj[1].primitives.resize(1);
  collision_obj[1].primitives[0].type = collision_obj[0].primitives[0].BOX;
  collision_obj[1].primitives[0].dimensions.resize(3);
  collision_obj[1].primitives[0].dimensions[0] = 0.4;
  collision_obj[1].primitives[0].dimensions[1] = 0.2;
  collision_obj[1].primitives[0].dimensions[2] = 0.4;
  collision_obj[1].primitive_poses.resize(1);
  collision_obj[1].primitive_poses[0].position.x = 0;
  collision_obj[1].primitive_poses[0].position.y = 0.5;
  collision_obj[1].primitive_poses[0].position.z = 0.2;
  collision_obj[1].operation = collision_obj[1].ADD;

  //define object
  collision_obj[2].id = "object";
  collision_obj[2].header.frame_id = "panda_link0";
  collision_obj[2].primitives.resize(1);
  collision_obj[2].primitives[0].type = collision_obj[0].primitives[0].BOX;
  collision_obj[2].primitives[0].dimensions.resize(3);
  collision_obj[2].primitives[0].dimensions[0] = 0.02;
  collision_obj[2].primitives[0].dimensions[1] = 0.02;
  collision_obj[2].primitives[0].dimensions[2] = 0.2;
  collision_obj[2].primitive_poses.resize(1);
  collision_obj[2].primitive_poses[0].position.x = 0.5;
  collision_obj[2].primitive_poses[0].position.y = 0;
  collision_obj[2].primitive_poses[0].position.z = 0.5;
  collision_obj[2].operation = collision_obj[2].ADD;

  current_scene.applyCollisionObjects(collision_obj);

}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
  //initiate grasps
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1); 

  //grasp pose
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.5;

  //define pre-grasp approach
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  //define post-grasp retreat
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;
  
  openGripper(grasps[0].pre_grasp_posture);
  closedGripper(grasps[0].grasp_posture);

  move_group.setSupportSurfaceName("table1");
  move_group.pick("object", grasps);

  
}


  void place(moveit::planning_interface::MoveGroupInterface& group)
{
  
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}





int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_pick_place");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  moveit::planning_interface::PlanningSceneInterface current_scene;
  
  //add collision object
  addCollisionObject(current_scene);

  //pick object
  pick(group);

  //place object
  place(group);
  
  return 0;
}

