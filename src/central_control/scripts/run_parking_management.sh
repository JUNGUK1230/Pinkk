#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
ROS_PC_IP="${ROS_PC_IP:-$(hostname -I | awk '{print $1}')}"
PINKY_HOST="${PINKY_HOST:-pinky@192.168.0.99}"
LOG_DIR="$PROJECT_ROOT/.runtime/parking_management"
WITH_PINKY=false
CHILD_PIDS=()

if [[ "${1:-}" == "--with-pinky" ]]; then
    WITH_PINKY=true
fi

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ERROR: ROS 2 Jazzy setup not found: $ROS_SETUP" >&2
    exit 1
fi
if [[ ! -x "$PROJECT_ROOT/.venv/bin/python" ]]; then
    echo "ERROR: create the project virtual environment first." >&2
    echo "  python3 -m venv --system-site-packages .venv" >&2
    echo "  .venv/bin/pip install -r requirements-vision.txt" >&2
    exit 1
fi

# shellcheck disable=SC1090
set +u
source "$ROS_SETUP"
set -u
export ROS_DOMAIN_ID
mkdir -p "$LOG_DIR"
cd "$PROJECT_ROOT"

port_in_use() {
    ss -ltn "sport = :$1" | tail -n +2 | grep -q .
}

require_free_port() {
    if port_in_use "$1"; then
        echo "ERROR: port $1 is already in use. Stop the previous server first." >&2
        exit 1
    fi
}

cleanup() {
    trap - INT TERM EXIT
    echo
    echo "Stopping parking management services..."
    for pid in "${CHILD_PIDS[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    wait "${CHILD_PIDS[@]}" 2>/dev/null || true
}

require_free_port 8000
require_free_port 8080
require_free_port 9090
trap cleanup INT TERM EXIT

ros2 run web_video_server web_video_server \
    >"$LOG_DIR/web_video_server.log" 2>&1 &
CHILD_PIDS+=("$!")

ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    >"$LOG_DIR/rosbridge.log" 2>&1 &
CHILD_PIDS+=("$!")

python3 -m http.server 8000 \
    --directory "$PROJECT_ROOT/src/central_control/parking_management_web" \
    >"$LOG_DIR/web.log" 2>&1 &
CHILD_PIDS+=("$!")

"$PROJECT_ROOT/.venv/bin/python" -m \
    src.central_control.overhead_vision.localization.live_localization \
    --no-display >"$LOG_DIR/localization.log" 2>&1 &
CHILD_PIDS+=("$!")

if $WITH_PINKY; then
    if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$PINKY_HOST" true; then
        echo "ERROR: --with-pinky requires SSH key authentication for $PINKY_HOST." >&2
        echo "Run without --with-pinky, or configure ssh-copy-id first." >&2
        exit 1
    fi
    ssh -o BatchMode=yes "$PINKY_HOST" \
        'source /opt/ros/jazzy/setup.bash && source ~/pinky_pro/install/setup.bash && export ROS_DOMAIN_ID=36 && exec ros2 launch pinky_bringup bringup_robot.launch.xml namespace:=pinky1' \
        >"$LOG_DIR/pinky_bringup.log" 2>&1 &
    CHILD_PIDS+=("$!")
fi

sleep 2
for pid in "${CHILD_PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "ERROR: a service exited during startup. Check logs in $LOG_DIR" >&2
        exit 1
    fi
done

URL="http://${ROS_PC_IP}:8000"
echo "Parking management services are running."
echo "Open: $URL"
echo "Logs: $LOG_DIR"
echo "Press Ctrl+C to stop all services started by this script."

if command -v xdg-open >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
    xdg-open "$URL" >/dev/null 2>&1 || true
fi

wait -n "${CHILD_PIDS[@]}"
echo "ERROR: a service stopped unexpectedly. Check logs in $LOG_DIR" >&2
exit 1
