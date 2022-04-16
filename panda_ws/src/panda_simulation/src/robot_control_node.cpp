#include <jsoncpp/json/json.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

static const std::string PLANNING_GROUP_ARM = "panda_arm";
static const std::string APP_DIRECTORY_NAME = ".panda_simulation";

moveit_msgs::CollisionObject extractObstacleFromJson(Json::Value &root, std::string name)
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = name;

  const Json::Value dimensions = root["dimensions"];
  ROS_INFO_STREAM("Extracted dimensions: " << dimensions);
  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = dimensions["x"].asDouble();
  primitive.dimensions[1] = dimensions["y"].asDouble();
  primitive.dimensions[2] = dimensions["z"].asDouble();

  const Json::Value position = root["position"];
  ROS_INFO_STREAM("Extracted position: " << position);

  const Json::Value orientation = root["orientation"];
  ROS_INFO_STREAM("Extracted orientation: " << orientation);
  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = orientation["w"].asDouble();
  box_pose.orientation.x = orientation["x"].asDouble();
  box_pose.orientation.y = orientation["y"].asDouble();
  box_pose.orientation.z = orientation["z"].asDouble();

  // MoveIt! planning scene expects the center of the object as position.
  // We add half of its dimension to its position
  box_pose.position.x = position["x"].asDouble() + primitive.dimensions[0] / 2.0;
  box_pose.position.y = position["y"].asDouble() + primitive.dimensions[1] / 2.0;
  box_pose.position.z = position["z"].asDouble() + primitive.dimensions[2] / 2.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return std::move(collision_object);
}

/////////////////////////////////////////////////////////////
std::vector<double> object_position = {0.5, 0.0, 0.5}; // default {0.5, 0.0, 0.5}

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

int main(int argc, char **argv)
{
  namespace fs = boost::filesystem;
  ROS_INFO("RUNNING robot_control_node");

  ros::init(argc, argv, "robot_control_node");

  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_arm(PLANNING_GROUP_ARM);

  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  moveit_msgs::PlanningScene planning_scene;

  // read JSON files from ~/.panda_simulation
  fs::path home(getenv("HOME"));
  if (fs::is_directory(home))
  {
    fs::path app_directory(home);
    app_directory /= APP_DIRECTORY_NAME;

    if (!fs::exists(app_directory) && !fs::is_directory(app_directory))
    {
      ROS_WARN_STREAM(app_directory << " does not exist");

      // Create .panda_simulation directory
      std::string path(getenv("HOME"));
      path += "/.panda_simulation";
      ROS_INFO("Creating %s collision objects directory.", path);
      try
      {
        boost::filesystem::create_directory(path);
      }
      catch (const std::exception&)
      {
        ROS_ERROR(
          "%s directory could not be created."
          "Please create this directory yourself "
          "if you want to specify collision objects.", path.c_str());
        return -1;
      }
    }

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    ROS_INFO_STREAM(app_directory << " is a directory containing:");
    for (auto &entry : boost::make_iterator_range(fs::directory_iterator(app_directory), {}))
    {
      ROS_INFO_STREAM(entry);

      std::ifstream file_stream(entry.path().string(), std::ifstream::binary);
      if (file_stream)
      {
        Json::Value root;
        file_stream >> root;

        moveit_msgs::CollisionObject collision_object = extractObstacleFromJson(root, entry.path().stem().string());
        collision_objects.push_back(collision_object);
      }
      else
      {
        ROS_WARN_STREAM("could not open file " << entry.path());
      }
    }

    // Publish the collision objects to the scene
    for (const auto &collision_object : collision_objects)
    {
      collision_object.header.frame_id = move_group_arm.getPlanningFrame();
      planning_scene.world.collision_objects.push_back(collision_object);
    }

    ROS_INFO_STREAM("# collision objects " << planning_scene.world.collision_objects.size());
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    ROS_INFO("robot_control_node is ready");

    /////////////////////// pick and place //////////////////////
    // some delay
    ros::WallDuration(2.0).sleep();

    // declare planning scene interface for add collision object
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    move_group_arm.setPlanningTime(45.0);

    addCollisionObjects(planning_scene_interface);

    ROS_INFO("Collision object added to the scene");

    // some delay
    ros::WallDuration(2.0).sleep();

    pick(move_group_arm);

    ros::WallDuration(1.0).sleep();

    ros::waitForShutdown();

    return 0;
  }
}