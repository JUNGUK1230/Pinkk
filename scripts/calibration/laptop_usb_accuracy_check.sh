#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

MODE="${1:-help}"
setup_ros_workspace

COMMON_ARGS=(
    --transit-z-mm 150
    --standoff-mm 100
    --angle-step-deg 180
    --final-z-step-mm 10
)

case "${MODE}" in
    observe)
        exec ros2 run pinkk_handeye_automation usb_pre_approach observe
        ;;
    observe-execute)
        echo "3초 후 초기 관측 자세로 이동하고 정지·자세 유지를 검증합니다"
        exec ros2 run pinkk_handeye_automation usb_pre_approach \
            observe --execute
        ;;
    show)
        exec ros2 run pinkk_handeye_automation usb_pre_approach show
        ;;
    check)
        exec ros2 run pinkk_handeye_automation usb_pre_approach \
            run "${COMMON_ARGS[@]}"
        ;;
    execute)
        echo "IK 사전검사 후 3초 대기하고 실제 정확도 검증 이동을 실행합니다"
        exec ros2 run pinkk_handeye_automation usb_pre_approach \
            run "${COMMON_ARGS[@]}" --execute
        ;;
    *)
        cat <<'EOF'
사용법:
  laptop_usb_accuracy_check.sh observe          # 초기 관측 자세 DRY RUN
  laptop_usb_accuracy_check.sh observe-execute  # 초기 관측 자세 실제 이동
  laptop_usb_accuracy_check.sh show     # 현재 USB/목표 TF만 출력
  laptop_usb_accuracy_check.sh check    # 이동 없는 전체 IK 검사
  laptop_usb_accuracy_check.sh execute  # 실제 좌표 정확도 검증 이동

순서:
  observe → observe-execute → 로봇 정지 → 검출 → check → execute
EOF
        exit 2
        ;;
esac
