#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_random_node");
    ros::init_options::AnonymousName;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface group("panda_arm");
    group.setRandomTarget();
    group.move();
    ros::waitForShutdown();
}