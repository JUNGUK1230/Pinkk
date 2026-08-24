#!/usr/bin/env bash
set -uo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
ERRORS=0
WARNINGS=0

ok() {
    echo "[OK] $*"
}

warn() {
    echo "[WARN] $*" >&2
    WARNINGS=$((WARNINGS + 1))
}

fail() {
    echo "[FAIL] $*" >&2
    ERRORS=$((ERRORS + 1))
}

require_file() {
    if [[ -f "$PROJECT_ROOT/$1" ]]; then
        ok "$1"
    else
        fail "missing file: $1"
    fi
}

require_command() {
    if command -v "$1" >/dev/null 2>&1; then
        ok "command: $1"
    else
        fail "missing command: $1"
    fi
}

echo "PINKK environment check"
echo "Project root: $PROJECT_ROOT"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ok "ROS 2 Jazzy"
else
    fail "ROS 2 Jazzy setup not found: /opt/ros/jazzy/setup.bash"
fi
require_command python3
require_command colcon
require_command ros2
require_command ssh
require_command scp

if [[ -x "$PROJECT_ROOT/.venv/bin/python" ]]; then
    ok "virtual environment: .venv"
    if MPLCONFIGDIR=/tmp/pinkk-matplotlib-cache \
        "$PROJECT_ROOT/.venv/bin/python" -c \
        'import cv2, flask, numpy, scipy, ultralytics, yaml; import rclpy' \
        >/dev/null 2>&1; then
        ok "Python runtime imports"
    else
        fail "Python imports failed; install requirements-vision.txt in .venv"
    fi
    CUDA_STATUS="$(MPLCONFIGDIR=/tmp/pinkk-matplotlib-cache \
        "$PROJECT_ROOT/.venv/bin/python" -c \
        'import torch; print("available" if torch.cuda.is_available() else "unavailable")' \
        2>/dev/null || echo unknown)"
    if [[ "$CUDA_STATUS" == available ]]; then
        ok "PyTorch CUDA"
    else
        warn "PyTorch CUDA unavailable; set inference_device: cpu or repair NVIDIA"
    fi
else
    fail "missing .venv; create it with --system-site-packages"
fi

require_file src/central_control/models/best.pt
require_file src/central_control/camera_tools/first_map/camera_calibration.npz
require_file src/central_control/camera_tools/first_map/bev_homography.npz
require_file src/central_control/camera_tools/first_map/camera_to_lidar_rigid_registration.npz
require_file src/central_control/camera_tools/first_map/my_test_map0710.png
require_file src/central_control/path_planning/output/fixed_route_manifest.csv

if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
    ok "colcon build output"
else
    fail "install/setup.bash missing; run colcon build --symlink-install"
fi

if command -v nvidia-smi >/dev/null 2>&1; then
    if nvidia-smi >/dev/null 2>&1; then
        ok "NVIDIA driver"
    else
        warn "NVIDIA GPU exists but nvidia-smi cannot access the driver"
    fi
fi

if command -v v4l2-ctl >/dev/null 2>&1; then
    ok "camera utility: v4l2-ctl"
else
    warn "v4l2-ctl missing; install v4l-utils to inspect camera devices"
fi

echo "Result: errors=$ERRORS warnings=$WARNINGS"
((ERRORS == 0))
