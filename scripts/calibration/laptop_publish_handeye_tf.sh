#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

SELECTOR="${1:-active}"
DATA_MANAGER="${SCRIPT_DIR}/handeye_data_manager.py"

setup_ros_workspace
read -r TX TY TZ QX QY QZ QW < <(
    /usr/bin/python3 "${DATA_MANAGER}" values "${SELECTOR}"
)
echo "Hand-eye static TF 발행: selector=${SELECTOR}"
exec ros2 run tf2_ros static_transform_publisher \
    --x "${TX}" \
    --y "${TY}" \
    --z "${TZ}" \
    --qx "${QX}" \
    --qy "${QY}" \
    --qz "${QZ}" \
    --qw "${QW}" \
    --frame-id joint6_flange \
    --child-frame-id camera_optical_frame
