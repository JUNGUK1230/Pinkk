#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

setup_ros_workspace
echo "현재 활성 Easy Handeye2 결과를 static TF로 발행합니다"
exec ros2 run tf2_ros static_transform_publisher \
    --x -0.032326655 \
    --y -0.040054972 \
    --z 0.030691236 \
    --qx -0.008700106 \
    --qy 0.002006141 \
    --qz -0.374364741 \
    --qw 0.927238548 \
    --frame-id joint6_flange \
    --child-frame-id camera_optical_frame
