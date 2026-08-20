#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

SHOW_PREVIEW="${1:-false}"
CAMERA="${2:-0}"
INTRINSICS_PATH="${3:-${REPO_ROOT}/src/robot_arm/calibration/camera_intrinsics/results/intrinsics.npz}"

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

if [[ ! -f "${INTRINSICS_PATH}" ]]; then
    echo "카메라 내부 보정 파일이 없습니다: ${INTRINSICS_PATH}" >&2
    exit 1
fi

echo "ChArUco TF 시작: preview=${SHOW_PREVIEW}, camera=${CAMERA}, intrinsics=${INTRINSICS_PATH}"
exec ros2 launch pinkk_mycobot_bridge charuco_tf_bridge.launch.py \
    show_preview:="${SHOW_PREVIEW}" \
    camera:="${CAMERA}" \
    intrinsics_path:="${INTRINSICS_PATH}"
