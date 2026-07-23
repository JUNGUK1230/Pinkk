#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

MODE="${1:-help}"
TARGET_SAMPLES="${2:-30}"
MINIMUM_SAMPLES="${3:-20}"
RUN_LABEL="${4:-auto_${TARGET_SAMPLES}_samples}"
DATA_MANAGER="${SCRIPT_DIR}/handeye_data_manager.py"

setup_handeye_workspace

case "${MODE}" in
    check)
        echo "이동 없는 IK DRY RUN을 시작합니다"
        exec ros2 launch pinkk_handeye_automation auto_calibrate.launch.py \
            execute:=false \
            target_samples:="${TARGET_SAMPLES}" \
            minimum_samples:="${MINIMUM_SAMPLES}"
        ;;
    execute)
        if (( TARGET_SAMPLES > 30 )); then
            echo "현재 자동 관측 자세는 최대 30개입니다. target은 30 이하로 지정하세요." >&2
            exit 2
        fi
        RUN_DIR="$(
            /usr/bin/python3 "${DATA_MANAGER}" create \
                --label "${RUN_LABEL}" \
                --target-samples "${TARGET_SAMPLES}" \
                --minimum-samples "${MINIMUM_SAMPLES}"
        )"
        echo "실제 자동 Hand-eye 수집을 시작합니다: target=${TARGET_SAMPLES}, minimum=${MINIMUM_SAMPLES}"
        echo "이번 실행 영구 보관 폴더: ${RUN_DIR}"
        set +e
        ros2 launch pinkk_handeye_automation auto_calibrate.launch.py \
            execute:=true \
            target_samples:="${TARGET_SAMPLES}" \
            minimum_samples:="${MINIMUM_SAMPLES}"
        ROS_STATUS=$?
        set -e
        ARCHIVE_STATUS=0
        /usr/bin/python3 "${DATA_MANAGER}" archive "${RUN_DIR}" \
            || ARCHIVE_STATUS=$?
        if (( ROS_STATUS != 0 )); then
            exit "${ROS_STATUS}"
        fi
        exit "${ARCHIVE_STATUS}"
        ;;
    *)
        cat <<'EOF'
사용법:
  laptop_auto_handeye.sh check [목표샘플] [최소샘플]
  laptop_auto_handeye.sh execute [목표샘플] [최소샘플] [실행이름]

예:
  laptop_auto_handeye.sh check 30 20
  laptop_auto_handeye.sh execute 30 20 bracket_fixed

execute가 끝나면 samples/calibration/metadata/matrix를 날짜별 run 폴더에 보관합니다.
기존 run은 덮어쓰지 않습니다.
EOF
        exit 2
        ;;
esac
