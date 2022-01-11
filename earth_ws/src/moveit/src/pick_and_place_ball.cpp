#include "ros/ros.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class MyPickPlace
{
private:
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::NodeHandle nh;
    geometry_msgs::Pose::ConstPtr goal_pose;
    // moveit::planning_interface::MoveGroupInterface group;

public:
    MyPickPlace()
    {
        sub = nh.subscribe("object_coor", 10, &MyPickPlace::object_coor_callback, this);
        // declare planning scene interface for add collision object
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        // moveit::planning_interface::MoveGroupInterface group_panda("panda_arm");
        // group = new moveit::planning_interface::MoveGroupInterface("panda_arm");
        // group->setPlanningTime(45.0);

        addCollisionObjects(planning_scene_interface);

        ROS_INFO("Collision object added to the scene");

        // some delay
        ros::WallDuration(1.0).sleep();

        // pick(*group);
    }

void object_coor_callback(const geometry_msgs::Pose::ConstPtr msg)
  {
      ROS_INFO("[CV_NODE]: x: [%f] y:[%f] z: [%f]", msg->position.x, msg->position.y, msg->position.z);
      ROS_INFO("[CV_NODE]: qx: [%f] qy:[%f] qz: [%f] qw:[%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
      goal_pose = msg;

      moveit::planning_interface::MoveGroupInterface group("panda_arm");
      group.setPlanningTime(45.0);
      pick(group);
      ros::WallDuration(5.0).sleep();
      place(group);
      ros::WallDuration(1.0).sleep();

  }

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
    //TODO: find another way to neglect not fully closed gripper
    posture.points[0].positions[0] = 0.015;
    posture.points[0].positions[1] = 0.015;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // the position for panda_link8 = 0.4 + (radius of sphere(0.02) + distance b/w panda_link8 and palm of eef (0.1))
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(M_PI, 0, -M_PI/4); //(180, 0, -45)
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = goal_pose->position.x;
    grasps[0].grasp_pose.pose.position.y = goal_pose->position.y;
    grasps[0].grasp_pose.pose.position.z = goal_pose->position.z;
    // grasps[0].grasp_pose.pose.position.x = 0.5;
    // grasps[0].grasp_pose.pose.position.y = 0;
    // grasps[0].grasp_pose.pose.position.z = 0.52;

    // Setting pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    // Direction is set as negative z axis
    grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
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

void place(moveit::planning_interface::MoveGroupInterface& group)
{
    // BEGIN_SUB_TUTORIAL place
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0); //(0, 0, 90)
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* While placing it is the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = 0.5;
    place_location[0].place_pose.pose.position.y = 0.0;
    place_location[0].place_pose.pose.position.z = 0.42;

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
    /* Direction is set as positive z axis */
    place_location[0].post_place_retreat.direction.vector.z = 1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    group.place("object1", place_location);
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
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].SPHERE;
    collision_objects[1].primitives[0].dimensions.resize(1);
    collision_objects[1].primitives[0].dimensions[0] = 0.02;

    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0.5;
    collision_objects[1].primitive_poses[0].position.y = 0;
    collision_objects[1].primitive_poses[0].position.z = 0.41;

    collision_objects[1].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);

}

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pick_and_place_ball");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // some delay
    ros::WallDuration(1.0).sleep();

    MyPickPlace myPickPlace;

    ros::waitForShutdown();
    return 0;
}
