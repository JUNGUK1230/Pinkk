#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
PYTHON="$PROJECT_ROOT/.venv/bin/python"
YOLO_SCRIPT="$PROJECT_ROOT/src/central_control/camera_tools/first_map/live_yolo_bev_map.py"

if [[ ! -x "$PYTHON" ]]; then
    echo "가상환경을 찾을 수 없습니다: $PYTHON" >&2
    echo "먼저 python3 -m venv .venv && .venv/bin/pip install -r requirements.txt 를 실행하세요." >&2
    exit 1
fi

if [[ ! -e /dev/video2 ]]; then
    echo "상단 카메라 /dev/video2를 찾을 수 없습니다." >&2
    echo "카메라를 다시 연결한 뒤 v4l2-ctl --list-devices로 확인하세요." >&2
    exit 1
fi

cd "$PROJECT_ROOT"
exec "$PYTHON" "$YOLO_SCRIPT"
