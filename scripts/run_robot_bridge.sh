#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /home/jetcobot/venv/mycobot/bin/activate
source /home/jetcobot/mycobot_moveit_ws/install_pinkk/setup.bash

export ROS_DOMAIN_ID=36
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

exec ros2 launch pinkk_usb_insertion observe_session.launch.py
