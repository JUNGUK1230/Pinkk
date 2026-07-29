#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /home/juwon/mycobot_moveit_ws/install/setup.bash
source /home/juwon/Desktop/Pinkk-robot-arm/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=36
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

exec ros2 launch pinkk_usb_insertion hybrid_alignment.launch.py
