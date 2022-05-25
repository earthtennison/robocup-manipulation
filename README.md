# RoboCup@Home CR3

## Build
`$ catkin_make -DCMAKE_BUILD_TYPE=Release`

## Run on real robot CR3
0. Set up hardware connection, connect an ethernet to robot controller box. <br>
Set ip address to 192.168.5.10 same network with the robot (192.168.5.6)
1. run `$ roslaunch cr3_moveit_config cr3_moveit_planning_execution.launch`
2. run `$ roslaunch tesr_ros_cr3_pkg controller.launch` <br>
to initialize cr3 controller (socket communication), publish robot state to Rviz
2. Move robot in Rviz, click plan and excecute. The real robot will move according to Rviz.
3. To close and open gripper run `$ rostopic echo /cr3_gripper_command true` in new terminal. <br>
**true** for closing gripper and **false** for opening gripper.

## Run on simulation
1. **cr3_moveit_3** is moveit config for simulation which already includes the gripper
2. **cr3_gripper_pkg** is gripper control package for simulation (still error)
3. **cr3_moveit_control** is high-level control for pick and place task
