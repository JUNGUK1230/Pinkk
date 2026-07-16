#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

STREAM_URL="${1:-http://192.168.6.1:5000/stream}"

setup_ros_workspace
cd "${REPO_ROOT}"
echo "USB 수동 SolvePnP 시작: ${STREAM_URL}"
exec python3 -m \
    src.robot_arm.robot_camera.handeye_calibration_1828.applications.manual_usb_tf \
    --url "${STREAM_URL}"
