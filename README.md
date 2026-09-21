Hand Robot — ROS 2 Jazzy

ROS 2 framework for a 14 joint robotic hand using ROS 2 Jazzy, MoveIt 2, custom ROS interfaces, and sensor-glove teleoperation.

Requirements
Ubuntu 24.04 LTS
ROS 2 Jazzy
MoveIt 2
colcon
C++ compiler / CMake

Source ROS 2 before building:

source /opt/ros/jazzy/setup.bash


The robot currently has 14 controllable joints, in this order:

base_to_base_1
base_to_base_2
base_to_base_3
base_to_base_4
base_to_base_thumb
base_to_middle_1
base_to_middle_2
base_to_middle_3
base_to_middle_4
middle_to_tip_1
middle_to_tip_2
middle_to_tip_3
middle_to_tip_4
thumb_base_to_middle

The command interface is:

hand_interfaces/msg/JointCommand

with:

float64[] positions

Build the workspace:

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

Source the workspace:

source install/setup.bash

For subsequent terminals:

source /opt/ros/jazzy/setup.bash
source ~/hand_robot/install/setup.bash

The MoveIt configuration can be launched using:

ros2 launch hand_moveit_config demo.launch.py

The teleoperation node is:

ros2 run hand_teleop glove_node

The current glove driver and kinematics are software placeholders pending integration with the physical glove hardware.

Testing

The joint command topic is:

/joint_command

Inspect it with:

ros2 topic echo /joint_command




Packages
hand_description — URDF/Xacro robot model and visualisation.
hand_moveit_config — MoveIt 2, SRDF, kinematics, joint limits, controllers and launch files.
hand_interfaces — Custom ROS 2 messages.
hand_commander — Receives joint commands and interfaces with MoveIt.
hand_teleop — Sensor-glove driver, kinematics and human-to-robot joint mapping.
