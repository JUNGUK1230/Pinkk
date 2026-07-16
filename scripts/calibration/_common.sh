#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"

source_required() {
    local file="$1"
    if [[ ! -f "${file}" ]]; then
        echo "필수 환경 파일이 없습니다: ${file}" >&2
        exit 1
    fi
    # ROS setup scripts can read unset variables, so temporarily disable nounset.
    set +u
    # shellcheck disable=SC1090
    source "${file}"
    set -u
}

source_optional() {
    local file="$1"
    if [[ -f "${file}" ]]; then
        set +u
        # shellcheck disable=SC1090
        source "${file}"
        set -u
    fi
}

setup_ros_network() {
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
}

setup_ros_workspace() {
    source_required "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
    source_required "${HOME}/mycobot_moveit_ws/install/setup.bash"
    source_optional "${HOME}/mycobot_moveit_ws/install_bridge/setup.bash"
    source_optional "${HOME}/mycobot_moveit_ws/install_pinkk/setup.bash"
    setup_ros_network
}

setup_handeye_workspace() {
    source_required "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
    source_required "${HOME}/easy_handeye2_ws/install/setup.bash"
    source_required "${HOME}/mycobot_moveit_ws/install/setup.bash"
    source_optional "${HOME}/mycobot_moveit_ws/install_bridge/setup.bash"
    source_optional "${HOME}/mycobot_moveit_ws/install_pinkk/setup.bash"
    setup_ros_network
}

require_command() {
    local command_name="$1"
    if ! command -v "${command_name}" >/dev/null 2>&1; then
        echo "명령을 찾을 수 없습니다: ${command_name}" >&2
        exit 1
    fi
}
