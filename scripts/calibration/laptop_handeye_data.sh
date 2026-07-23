#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DATA_MANAGER="${SCRIPT_DIR}/handeye_data_manager.py"

COMMAND="${1:-help}"

case "${COMMAND}" in
    list)
        exec /usr/bin/python3 "${DATA_MANAGER}" list
        ;;
    activate)
        if [[ $# -lt 2 ]]; then
            echo "사용법: laptop_handeye_data.sh activate RUN" >&2
            exit 2
        fi
        exec /usr/bin/python3 "${DATA_MANAGER}" activate "$2"
        ;;
    show-active)
        exec /usr/bin/python3 "${DATA_MANAGER}" show-active
        ;;
    archive)
        if [[ $# -lt 2 ]]; then
            echo "사용법: laptop_handeye_data.sh archive RUN [EASY_NAME]" >&2
            exit 2
        fi
        if [[ $# -ge 3 ]]; then
            exec /usr/bin/python3 "${DATA_MANAGER}" archive "$2" --easy-name "$3"
        fi
        exec /usr/bin/python3 "${DATA_MANAGER}" archive "$2"
        ;;
    compute)
        if [[ $# -lt 2 ]]; then
            echo "사용법: laptop_handeye_data.sh compute RUN" >&2
            exit 2
        fi
        # Ignore ~/.local OpenCV packages: ROS Jazzy's apt OpenCV contains
        # calibrateHandEye and matches Easy Handeye2's runtime.
        export PYTHONNOUSERSITE=1
        exec /usr/bin/python3 "${DATA_MANAGER}" compute-run "$2"
        ;;
    *)
        cat <<'EOF'
Hand-eye 데이터 관리:
  laptop_handeye_data.sh list
  laptop_handeye_data.sh show-active
  laptop_handeye_data.sh activate RUN
  laptop_handeye_data.sh archive RUN [EASY_NAME]
  laptop_handeye_data.sh compute RUN

RUN은 전체 폴더 이름 또는 유일하게 구분되는 일부 문자열을 사용할 수 있습니다.
activate는 active 파일, 기존 호환 NPY, USB 시스템 handeye.yaml을 함께 갱신합니다.
compute는 samples_only run을 ROS 시스템 OpenCV/Tsai-Lenz로 복구 계산합니다.
EOF
        exit 2
        ;;
esac
