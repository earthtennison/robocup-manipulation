// trajectory action client ref https://wiki.ros.org/Robots/ARI/Joint%20Trajectory%20Controller

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/topic.h>
#include "std_msgs/Bool.h"

// Our Action interface type for moving ARI's head, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> hand_control_client;
typedef boost::shared_ptr<hand_control_client> hand_control_client_Ptr;

class SimpleGripperControl
{
private:
    ros::Subscriber sub;
    ros::NodeHandle nh;
    hand_control_client_Ptr HandClient;
    control_msgs::FollowJointTrajectoryGoal goal;

public:
    // constructor
    SimpleGripperControl()
    {
        sub = nh.subscribe("/gripper_command", 10, &SimpleGripperControl::gripper_cb, this);
        ROS_INFO("Starting simple_gripper_control node...");

        // Precondition: Valid clock

        if (!ros::Time::waitForValid(ros::WallDuration(10.0))) // NOTE: Important when using simulated clock
        {
            ROS_FATAL("Timed-out waiting for valid time.");
        }

        // Create an hand controller action client to close gripper
        createHandClient();
    }

    void gripper_cb(const std_msgs::Bool &msg)
    {
        // 1 is close, 0 is open
        if (msg.data)
        {
            // Generates the goal for the ARI's right arm
            close_gripper();
            ROS_INFO("Closign gripper...");

            // Sends the command to start the given trajectory 1s from now
            goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
            HandClient->sendGoal(goal);

            // Wait for trajectory execution
            while (!(HandClient->getState().isDone()) && ros::ok())
            {
                ros::Duration(3).sleep();
            }
        }
        else
        {
            // Generates the goal for the ARI's right arm
            open_gripper();
            ROS_INFO("Opening gripper...");

            // Sends the command to start the given trajectory 1s from now
            goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
            HandClient->sendGoal(goal);

            // Wait for trajectory execution
            while (!(HandClient->getState().isDone()) && ros::ok())
            {
                ros::Duration(3).sleep();
            }
        }
    }
    // Create a ROS action client to move ARI's right arm
    void createHandClient()
    {
        ROS_INFO("Creating action client to hand controller ...");

        HandClient.reset(new hand_control_client("/panda_hand_controller/follow_joint_trajectory"));

        int iterations = 0, max_iterations = 3;
        // Wait for hand controller action server to come up
        while (!HandClient->waitForServer(ros::Duration(2.0)) && ros::ok() && iterations < max_iterations)
        {
            ROS_DEBUG("Waiting for the panda_hand_controller server to come up");
            ++iterations;
        }

        if (iterations == max_iterations)
            throw std::runtime_error("Error in createHandClient: hand controller action server not available");
    }

    // Generates a simple trajectory with two waypoints to move ARI's arm
    void close_gripper()
    {
        goal.trajectory.joint_names.clear();
        // The joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("panda_finger_joint1");
        goal.trajectory.joint_names.push_back("panda_finger_joint2");

        // Two waypoints in this goal trajectory
        goal.trajectory.points.resize(1);

        // First trajectory point
        // Positions
        int index = 0;
        goal.trajectory.points[index].positions.resize(2);
        goal.trajectory.points[index].positions[0] = 0.0;
        goal.trajectory.points[index].positions[1] = 0.0;

        // Velocities
        goal.trajectory.points[index].velocities.resize(2);
        for (int j = 0; j < 2; ++j)
        {
            goal.trajectory.points[index].velocities[j] = 0.0;
        }
        // To be reached 6 seconds after starting along the trajectory
        goal.trajectory.points[index].time_from_start = ros::Duration(3.0);
    }

    void open_gripper()
    {
        goal.trajectory.joint_names.clear();
        // The joint names, which apply to all waypoints
        goal.trajectory.joint_names.push_back("panda_finger_joint1");
        goal.trajectory.joint_names.push_back("panda_finger_joint2");

        // Two waypoints in this goal trajectory
        goal.trajectory.points.resize(1);

        // First trajectory point
        // Positions
        int index = 0;
        goal.trajectory.points[index].positions.resize(2);
        goal.trajectory.points[index].positions[0] = 0.04;
        goal.trajectory.points[index].positions[1] = 0.04;

        // Velocities
        goal.trajectory.points[index].velocities.resize(2);
        for (int j = 0; j < 2; ++j)
        {
            goal.trajectory.points[index].velocities[j] = 0.0;
        }
        // To be reached 6 seconds after starting along the trajectory
        goal.trajectory.points[index].time_from_start = ros::Duration(3.0);
    }
};

// Entry point
int main(int argc, char **argv)
{
    // Init the ROS node
    ros::init(argc, argv, "simple_gripper_control");
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // some delay
    ros::WallDuration(1.0).sleep();

    SimpleGripperControl controller;

    ros::waitForShutdown();

    return 0;
}