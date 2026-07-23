#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

MODE="${1:-help}"
TARGET_VALID_POSES="${2:-15}"
DATA_MANAGER="${SCRIPT_DIR}/handeye_data_manager.py"
OLD_SELECTOR="${3:-20260715_baseline_old}"
NEW_SELECTOR="${4:-20260723_auto_30samples}"
MEASUREMENT_COUNT="${5:-5}"
OUTPUT_CSV="${6:-}"
POSE_LIMIT=30

if ! [[ "${TARGET_VALID_POSES}" =~ ^[0-9]+$ ]] \
    || (( TARGET_VALID_POSES < 1 || TARGET_VALID_POSES > POSE_LIMIT )); then
    echo "유효 자세 목표는 1~${POSE_LIMIT} 범위여야 합니다" >&2
    exit 2
fi
if ! [[ "${MEASUREMENT_COUNT}" =~ ^[0-9]+$ ]] \
    || (( MEASUREMENT_COUNT < 1 )); then
    echo "자세당 측정 횟수는 1 이상이어야 합니다" >&2
    exit 2
fi

setup_handeye_workspace
OLD_CALIB="$(/usr/bin/python3 "${DATA_MANAGER}" resolve "${OLD_SELECTOR}")"
NEW_CALIB="$(/usr/bin/python3 "${DATA_MANAGER}" resolve "${NEW_SELECTOR}")"

COMMON_ARGS=(
    old_calib_path:="${OLD_CALIB}"
    new_calib_path:="${NEW_CALIB}"
    pose_limit:="${POSE_LIMIT}"
    target_valid_poses:="${TARGET_VALID_POSES}"
    measurement_count:="${MEASUREMENT_COUNT}"
)
if [[ -n "${OUTPUT_CSV}" ]]; then
    COMMON_ARGS+=(output_csv:="${OUTPUT_CSV}")
fi

case "${MODE}" in
    check)
        echo "이동 없는 old/new calibration IK 비교 검사를 시작합니다"
        echo "후보=${POSE_LIMIT}, 목표 유효 자세=${TARGET_VALID_POSES}, 자세당 측정=${MEASUREMENT_COUNT}"
        exec ros2 launch pinkk_handeye_automation compare_calibrations.launch.py \
            execute:=false \
            "${COMMON_ARGS[@]}"
        ;;
    execute)
        MANAGED_OUTPUT=false
        if [[ -z "${OUTPUT_CSV}" ]]; then
            OUTPUT_CSV="$(
                /usr/bin/python3 "${DATA_MANAGER}" create-comparison \
                    --old "${OLD_CALIB}" \
                    --new "${NEW_CALIB}" \
                    --pose-limit "${POSE_LIMIT}" \
                    --target-valid-poses "${TARGET_VALID_POSES}" \
                    --measurement-count "${MEASUREMENT_COUNT}"
            )"
            COMMON_ARGS+=(output_csv:="${OUTPUT_CSV}")
            MANAGED_OUTPUT=true
        fi
        echo "실제 old/new calibration 자세 비교를 시작합니다"
        echo "후보=${POSE_LIMIT}, 목표 유효 자세=${TARGET_VALID_POSES}, 자세당 측정=${MEASUREMENT_COUNT}"
        echo "OLD: ${OLD_CALIB}"
        echo "NEW: ${NEW_CALIB}"
        echo "비교 결과 영구 보관 경로: ${OUTPUT_CSV}"
        echo "ChArUco 보드는 움직이지 말고, 기존 Hand-eye TF publisher는 종료하세요."
        set +e
        ros2 launch pinkk_handeye_automation compare_calibrations.launch.py \
            execute:=true \
            "${COMMON_ARGS[@]}"
        ROS_STATUS=$?
        set -e
        FINALIZE_STATUS=0
        if [[ "${MANAGED_OUTPUT}" == true ]]; then
            /usr/bin/python3 "${DATA_MANAGER}" finalize-comparison "${OUTPUT_CSV}" \
                || FINALIZE_STATUS=$?
        fi
        if (( ROS_STATUS != 0 )); then
            exit "${ROS_STATUS}"
        fi
        exit "${FINALIZE_STATUS}"
        ;;
    *)
        cat <<'EOF'
사용법:
  laptop_compare_handeye.sh check [목표_유효자세] [OLD_RUN] [NEW_RUN] [자세당_측정횟수]
  laptop_compare_handeye.sh execute [목표_유효자세] [OLD_RUN] [NEW_RUN] [자세당_측정횟수] [출력.csv]

기본 비교:
  laptop_compare_handeye.sh check 15 OLD_RUN NEW_RUN 5
  laptop_compare_handeye.sh execute 15 OLD_RUN NEW_RUN 5

RUN에는 data/runs 아래 폴더 이름, 일부 이름 또는 직접 .calib 경로를 사용할 수 있습니다.
check는 로봇을 움직이지 않고 IK만 검사합니다.
execute는 최대 30개 후보 중 유효 자세 목표를 채우면 즉시 종료합니다.
execute 출력 경로를 생략하면 data/comparisons 아래에 영구 저장됩니다.
EOF
        exit 2
        ;;
esac
