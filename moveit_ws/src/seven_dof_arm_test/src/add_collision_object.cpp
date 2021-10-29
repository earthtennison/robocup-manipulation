#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_collision_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface current_scene;
    sleep(5.0);

    moveit_msgs::CollisionObject cylinder;
    cylinder.header.frame_id = "panda_link0";
    cylinder.id = "obstacle_cylinder";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.6;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.2;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.0;
    pose.position.y = -0.4;
    pose.position.z = -0.4;

    cylinder.primitives.push_back(primitive);
    cylinder.primitive_poses.push_back(pose);
    cylinder.operation = cylinder.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(cylinder);

    current_scene.addCollisionObjects(collision_objects);




}