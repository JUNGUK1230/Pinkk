#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

source_required "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source_required "${HOME}/easy_handeye2_ws/install/setup.bash"
source_required "${HOME}/mycobot_moveit_ws/install/setup.bash"

cd "${HOME}/mycobot_moveit_ws"
echo "노트북용 Pinkk ROS 패키지를 install_pinkk에 빌드합니다"
exec colcon --log-base log_pinkk build \
    --build-base build_pinkk \
    --install-base install_pinkk \
    --base-paths \
        "${REPO_ROOT}/ros2_ws/src/pinkk_mycobot_bridge" \
        "${REPO_ROOT}/ros2_ws/src/pinkk_handeye_automation" \
        "${REPO_ROOT}/ros2_ws/src/pinkk_usb_insertion_interfaces" \
        "${REPO_ROOT}/ros2_ws/src/pinkk_usb_insertion" \
    --packages-select \
        pinkk_mycobot_bridge \
        pinkk_handeye_automation \
        pinkk_usb_insertion_interfaces \
        pinkk_usb_insertion
