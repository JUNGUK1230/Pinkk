#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
ROBOT_PROFILE="${1:-${PINKK_ROBOT_PROFILE:-robot_a}}"
case "${ROBOT_PROFILE}" in
    robot_a) PROFILE_DOMAIN_ID=36 ;;
    robot_b) PROFILE_DOMAIN_ID=38 ;;
    *)
        echo "지원하지 않는 로봇 프로필입니다: ${ROBOT_PROFILE} (robot_a|robot_b)" >&2
        exit 2
        ;;
esac

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

export ROS_DOMAIN_ID="${PINKK_ROS_DOMAIN_ID:-${PROFILE_DOMAIN_ID}}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY

echo "통합 명령 프로필=${ROBOT_PROFILE}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

exec ros2 topic pub --once \
    /robot_arm/frozen_target/command \
    std_msgs/msg/String \
    "{data: execute_full_sequence_with_final_z}"
