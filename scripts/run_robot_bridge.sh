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
source_environment /home/jetcobot/venv/mycobot/bin/activate
source_environment /home/jetcobot/mycobot_moveit_ws/install_pinkk/setup.bash

export ROS_DOMAIN_ID=36
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

exec ros2 launch pinkk_usb_insertion observe_session.launch.py
