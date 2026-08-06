#!/usr/bin/env bash
set -euo pipefail

source_environment() {
    local setup_file="$1"
    if [[ ! -f "${setup_file}" ]]; then
        echo "환경 파일이 없습니다: ${setup_file}" >&2
        exit 1
    fi
    set +u
    # shellcheck disable=SC1090
    source "${setup_file}"
    set -u
}

source_environment /opt/ros/jazzy/setup.bash
source_environment /home/juwon/mycobot_moveit_ws/install/setup.bash
source_environment /home/juwon/Desktop/Pinkk-robot-arm/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

exec ros2 launch pinkk_usb_insertion hybrid_alignment.launch.py
