#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

SPEED="${1:-50}"
GOAL_TOLERANCE_DEG="${2:-5.0}"
CARTESIAN_EXECUTION_ENABLED="${3:-false}"
CARTESIAN_MAX_TRANSLATION_M="${4:-0.0105}"
JOINT_EXECUTION_ENABLED="${5:-false}"
JOINT_MAX_COMMAND_ATTEMPTS="${6:-1}"

source_required "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source_required "${HOME}/venv/mycobot/bin/activate"
source_required "${HOME}/mycobot_moveit_ws/install/setup.bash"
source_optional "${HOME}/mycobot_moveit_ws/install_bridge/setup.bash"
source_optional "${HOME}/mycobot_moveit_ws/install_pinkk/setup.bash"

PYMYCOBOT_SITE="${HOME}/venv/mycobot/lib/python3.12/site-packages"
if [[ -d "${PYMYCOBOT_SITE}" ]]; then
    export PYTHONPATH="${PYMYCOBOT_SITE}${PYTHONPATH:+:${PYTHONPATH}}"
fi
setup_ros_network

echo "로봇 bridge 시작: speed=${SPEED}, tolerance=${GOAL_TOLERANCE_DEG}deg, joint_enabled=${JOINT_EXECUTION_ENABLED}, joint_attempts=${JOINT_MAX_COMMAND_ATTEMPTS}, cartesian_enabled=${CARTESIAN_EXECUTION_ENABLED}, cartesian_max=${CARTESIAN_MAX_TRANSLATION_M}m"
exec ros2 launch pinkk_mycobot_bridge trajectory_bridge.launch.py \
    speed:="${SPEED}" \
    goal_tolerance_deg:="${GOAL_TOLERANCE_DEG}" \
    joint_execution_enabled:="${JOINT_EXECUTION_ENABLED}" \
    joint_max_command_attempts:="${JOINT_MAX_COMMAND_ATTEMPTS}" \
    cartesian_execution_enabled:="${CARTESIAN_EXECUTION_ENABLED}" \
    cartesian_max_translation_m:="${CARTESIAN_MAX_TRANSLATION_M}"
