#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

setup_ros_workspace
echo "노트북 MoveIt/RViz 실제 실행 구성을 시작합니다"
exec ros2 launch pinkk_mycobot_bridge real_execution.launch.py
