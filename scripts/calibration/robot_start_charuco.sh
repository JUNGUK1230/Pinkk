#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

SHOW_PREVIEW="${1:-false}"

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

echo "ChArUco TF 시작: preview=${SHOW_PREVIEW}"
exec ros2 launch pinkk_mycobot_bridge charuco_tf_bridge.launch.py \
    show_preview:="${SHOW_PREVIEW}"
