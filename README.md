# robocup-manipulation

This repo is used to test movegroup interface to pick and place task in rviz, but not include in gazebo.

## Setup
0. build libfranka from this [tutorial](https://frankaemika.github.io/docs/installation_linux.html)

## Control arm from another node
1. launch rviz and load robot model `$ roslaunch panda_moveit_config demo.launch`
2. run movegroup interface node `$ roslaunch moveit move_group_interface.launch` <br> or `$ roslaunch moveit pick_and_place_ball.launch`
3. run demo computer vision node to send object pose to movegroup `roslaunch moveit cv_node.launch` <br>
you can type the pose value in x y z roll pitch yaw in the terminal

See the result in this [video](https://youtu.be/9xpJnPOwXso) <br>
Noted: Now there is an error in rviz and gazebo because the updated franka_description.

## Looping pick and place
1. launch `$ roslaunch moveit pick_and_place.launch`


See the result in this [video](https://youtu.be/IBU5n6xIhps)


