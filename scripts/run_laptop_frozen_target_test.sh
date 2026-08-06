#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

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
source_environment "${HOME}/mycobot_moveit_ws/install/setup.bash"
source_environment "${REPO_ROOT}/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY

exec ros2 launch pinkk_usb_insertion frozen_target_alignment.launch.py
