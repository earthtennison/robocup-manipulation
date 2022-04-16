#include "ros/ros.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

/////////////////////////////////////////////////////////////
std::vector<double> object_position = {0.5, 0.0, 0.5};

////////////////////////////////////////////////////////////

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    // object width is 0.02
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // the position for panda_link8 = 0.5 - (length of cube/2 - distance b/w panda_link8 and palm of eef (0.058) - some extra padding)
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI/2, -M_PI/4, -M_PI/2); //(-90, -45, -90)
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = object_position[0] - 0.085;
    grasps[0].grasp_pose.pose.position.y = object_position[1];
    grasps[0].grasp_pose.pose.position.z = object_position[2];

    // Setting pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    // Direction is set as positive x axis
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    // Direction is set as positive z axis
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // open gripper pose
    openGripper(grasps[0].pre_grasp_posture);

    // close gripper pose
    closedGripper(grasps[0].grasp_posture);

    // Set support surface as table1.
    move_group.setSupportSurfaceName("table1");

    // pick it
    move_group.pick("object1", grasps);


}


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    // create vector to hold 3 object
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    // table1
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "panda_link0";

    // table1 dimension
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    // table1 poses
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;

    collision_objects[0].operation = collision_objects[0].ADD;


    // object
    collision_objects[1].header.frame_id = "panda_link0";
    collision_objects[1].id = "object1";

    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.02;
    collision_objects[1].primitives[0].dimensions[1] = 0.02;
    collision_objects[1].primitives[0].dimensions[2] = 0.2;

    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = object_position[0];
    collision_objects[1].primitive_poses[0].position.y = object_position[1];
    collision_objects[1].primitive_poses[0].position.z = object_position[2];

    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_with_constraint");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // some delay
    ros::WallDuration(2.0).sleep();

    // declare planning scene interface for add collision object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setPlanningTime(45.0);

    addCollisionObjects(planning_scene_interface);

    ROS_INFO("Collision object added to the scene");

    // some delay
    ros::WallDuration(2.0).sleep();

    int count = 0;

    pick(group);

    ROS_INFO("Object picked at table 1 round %d", count);

    ros::WallDuration(1.0).sleep();

    ros::waitForShutdown();
    return 0;
}
