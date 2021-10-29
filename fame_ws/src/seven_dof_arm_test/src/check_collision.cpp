#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "check_collision_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    

    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    
    if (collision_result.collision == true)
    {
        ROS_INFO_STREAM("1. Self collision Test: " << "in self collision");
    }
    else{
        ROS_INFO_STREAM("1. Self collision Test: not in self collision");
    }

    collision_result.clear();
    // perform full collision test
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene.getCurrentState();
    planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("6. Full collision Test: " << (collision_result.collision ? "in" : "not in") << " collision");
    
}