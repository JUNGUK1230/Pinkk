#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
source "${SCRIPT_DIR}/_common.sh"

MODE="${1:-help}"
TARGET_SAMPLES="${2:-20}"
MINIMUM_SAMPLES="${3:-15}"

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
        echo "실제 자동 Hand-eye 수집을 시작합니다: target=${TARGET_SAMPLES}, minimum=${MINIMUM_SAMPLES}"
        exec ros2 launch pinkk_handeye_automation auto_calibrate.launch.py \
            execute:=true \
            target_samples:="${TARGET_SAMPLES}" \
            minimum_samples:="${MINIMUM_SAMPLES}"
        ;;
    *)
        cat <<'EOF'
사용법:
  laptop_auto_handeye.sh check [목표샘플] [최소샘플]
  laptop_auto_handeye.sh execute [목표샘플] [최소샘플]

예:
  laptop_auto_handeye.sh check 20 15
  laptop_auto_handeye.sh execute 20 15
EOF
        exit 2
        ;;
esac
