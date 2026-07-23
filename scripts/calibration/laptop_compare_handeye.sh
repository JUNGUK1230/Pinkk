#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

MODE="${1:-help}"
POSE_LIMIT="${2:-30}"
DATA_DIR="${REPO_ROOT}/src/robot_arm/robot_camera/handeye_calibration_1828/data/easy_handeye2"
OLD_CALIB="${3:-${DATA_DIR}/pinkk_eye_in_hand_20260715.calib}"
NEW_CALIB="${4:-${DATA_DIR}/pinkk_eye_in_hand_30samples_20260723.calib}"
OUTPUT_CSV="${5:-}"

setup_handeye_workspace

COMMON_ARGS=(
    old_calib_path:="${OLD_CALIB}"
    new_calib_path:="${NEW_CALIB}"
    pose_limit:="${POSE_LIMIT}"
)
if [[ -n "${OUTPUT_CSV}" ]]; then
    COMMON_ARGS+=(output_csv:="${OUTPUT_CSV}")
fi

case "${MODE}" in
    check)
        echo "이동 없는 old/new calibration IK 비교 검사를 시작합니다: poses=${POSE_LIMIT}"
        exec ros2 launch pinkk_handeye_automation compare_calibrations.launch.py \
            execute:=false \
            "${COMMON_ARGS[@]}"
        ;;
    execute)
        echo "실제 old/new calibration 자세 비교를 시작합니다: poses=${POSE_LIMIT}"
        echo "ChArUco 보드는 움직이지 말고, 기존 Hand-eye TF publisher는 종료하세요."
        exec ros2 launch pinkk_handeye_automation compare_calibrations.launch.py \
            execute:=true \
            "${COMMON_ARGS[@]}"
        ;;
    *)
        cat <<'EOF'
사용법:
  laptop_compare_handeye.sh check [자세수] [OLD.calib] [NEW.calib] [출력.csv]
  laptop_compare_handeye.sh execute [자세수] [OLD.calib] [NEW.calib] [출력.csv]

기본 비교:
  laptop_compare_handeye.sh check 30
  laptop_compare_handeye.sh execute 30

check는 로봇을 움직이지 않고 IK만 검사합니다.
execute 결과는 출력 경로를 생략하면 ~/handeye_comparison_날짜_시간.csv와
같은 이름의 .summary.json으로 저장됩니다.
EOF
        exit 2
        ;;
esac
