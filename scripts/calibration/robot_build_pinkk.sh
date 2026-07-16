#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

source_required "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
source_required "${HOME}/venv/mycobot/bin/activate"
source_required "${HOME}/mycobot_moveit_ws/install/setup.bash"

cd "${HOME}/mycobot_moveit_ws"
echo "로봇 PC용 Pinkk bridge를 install_pinkk에 빌드합니다"
exec colcon --log-base log_pinkk build \
    --build-base build_pinkk \
    --install-base install_pinkk \
    --base-paths "${REPO_ROOT}/ros2_ws/src/pinkk_mycobot_bridge" \
    --packages-select pinkk_mycobot_bridge
