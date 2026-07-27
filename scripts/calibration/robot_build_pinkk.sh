#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

source_required "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source_required "${HOME}/venv/mycobot/bin/activate"
source_required "${HOME}/mycobot_moveit_ws/install/setup.bash"

# mycobot 가상환경은 pymycobot 실행에 필요하지만 ROS Jazzy의 rosidl_adapter가
# 사용하는 Debian empy 모듈도 함께 볼 수 있어야 interface 빌드가 가능하다.
SYSTEM_PYTHON_DIST="/usr/lib/python3/dist-packages"
if [[ -d "${SYSTEM_PYTHON_DIST}" ]]; then
    export PYTHONPATH="${SYSTEM_PYTHON_DIST}${PYTHONPATH:+:${PYTHONPATH}}"
fi

cd "${HOME}/mycobot_moveit_ws"
echo "로봇 PC용 Pinkk interface와 bridge를 install_pinkk에 빌드합니다"
exec colcon --log-base log_pinkk build \
    --build-base build_pinkk \
    --install-base install_pinkk \
    --base-paths \
        "${REPO_ROOT}/ros2_ws/src/pinkk_usb_insertion_interfaces" \
        "${REPO_ROOT}/ros2_ws/src/pinkk_mycobot_bridge" \
    --packages-select \
        pinkk_usb_insertion_interfaces \
        pinkk_mycobot_bridge
