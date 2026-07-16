#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

setup_handeye_workspace
echo "Easy Handeye2 서버 시작: pinkk_eye_in_hand"
echo "계산이 끝날 때까지 기존 Hand-eye static TF는 따로 실행하지 마세요"
exec ros2 launch easy_handeye2 calibrate.launch.py \
    name:=pinkk_eye_in_hand \
    calibration_type:=eye_in_hand \
    robot_base_frame:=g_base \
    robot_effector_frame:=joint6_flange \
    tracking_base_frame:=camera_optical_frame \
    tracking_marker_frame:=charuco_board
