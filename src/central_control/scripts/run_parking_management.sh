#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
ROS_PC_IP="${ROS_PC_IP:-$(hostname -I | awk '{print $1}')}"
PINKY1_HOST="${PINKY1_HOST:-${PINKY_HOST:-pinky@192.168.0.99}}"
# PINKY_02는 192.168.0.103을 기본 주소로 사용한다.
PINKY2_HOST="${PINKY2_HOST:-pinky@192.168.0.103}"
LOG_DIR="$PROJECT_ROOT/.runtime/parking_management"
WITH_PINKY=true
WITHOUT_CAMERA=false
SETUP_SSH=false
REMOTE_PINKY_TARGETS=()
CHILD_PIDS=()

for option in "$@"; do
    case "$option" in
        --with-pinky)
            WITH_PINKY=true
            ;;
        --without-pinky)
            WITH_PINKY=false
            ;;
        --without-camera)
            WITHOUT_CAMERA=true
            ;;
        --setup-ssh)
            SETUP_SSH=true
            ;;
        *)
            echo "ERROR: unknown option: $option" >&2
            echo "Usage: $0 [--setup-ssh] [--without-camera] [--with-pinky|--without-pinky]" >&2
            exit 2
            ;;
    esac
done

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

if $SETUP_SSH; then
    if ! $WITH_PINKY; then
        echo "ERROR: --setup-ssh cannot be used with --without-pinky." >&2
        exit 2
    fi
    if ! command -v ssh-copy-id >/dev/null 2>&1; then
        echo "ERROR: ssh-copy-id is required to register Pinky SSH keys." >&2
        exit 1
    fi
    echo "Registering the central PC SSH key on PINKY_01 ($PINKY1_HOST)..."
    ssh-copy-id -o StrictHostKeyChecking=accept-new "$PINKY1_HOST"
    echo "Registering the central PC SSH key on PINKY_02 ($PINKY2_HOST)..."
    ssh-copy-id -o StrictHostKeyChecking=accept-new "$PINKY2_HOST"
    echo "SSH key registration completed for both Pinky vehicles."
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
    for target in "${REMOTE_PINKY_TARGETS[@]}"; do
        host="${target%%|*}"
        namespace="${target#*|}"
        ssh -o BatchMode=yes -o ConnectTimeout=3 "$host" \
            "pid=\$(pgrep -f \"^bash /home/pinky/run_pinky_services.sh $namespace\$\" | head -n 1); [[ -z \"\$pid\" ]] || kill -TERM \"\$pid\"" \
            >/dev/null 2>&1 || true
    done
    for pid in "${CHILD_PIDS[@]}"; do
        # Each service is started in its own process group with setsid.
        # Stop the entire group so ROS launch children cannot keep ports open.
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    wait "${CHILD_PIDS[@]}" 2>/dev/null || true
    for pid in "${CHILD_PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
}

require_free_port 8000
require_free_port 9090
if ! $WITHOUT_CAMERA; then
    require_free_port 8080
fi
trap cleanup INT TERM EXIT

if ! $WITHOUT_CAMERA; then
    setsid ros2 run web_video_server web_video_server \
        >"$LOG_DIR/web_video_server.log" 2>&1 &
    CHILD_PIDS+=("$!")
fi

setsid ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    >"$LOG_DIR/rosbridge.log" 2>&1 &
CHILD_PIDS+=("$!")

setsid python3 "$PROJECT_ROOT/src/central_control/scripts/serve_parking_management.py" \
    --port 8000 \
    --directory "$PROJECT_ROOT/src/central_control/parking_management_web" \
    --vehicle-config "$PROJECT_ROOT/src/central_control/config/vehicles.yaml" \
    >"$LOG_DIR/web.log" 2>&1 &
CHILD_PIDS+=("$!")

if ! $WITHOUT_CAMERA; then
    setsid "$PROJECT_ROOT/.venv/bin/python" -m \
        src.central_control.overhead_vision.localization.live_localization \
        >"$LOG_DIR/localization.log" 2>&1 &
    CHILD_PIDS+=("$!")
fi

start_remote_pinky() {
    local host="$1"
    local namespace="$2"
    local controller_id="$3"
    if ! ssh -o BatchMode=yes -o ConnectTimeout=5 \
        -o ServerAliveInterval=2 -o ServerAliveCountMax=3 \
        "$host" true; then
        echo "ERROR: Pinky SSH key authentication failed for $host ($namespace)." >&2
        echo "Configure ssh-copy-id, or run with --without-pinky." >&2
        return 1
    fi
    scp -q -o BatchMode=yes -o ConnectTimeout=5 \
        -o ServerAliveInterval=2 -o ServerAliveCountMax=3 \
        "$PROJECT_ROOT/src/vehicle_control/pinky_status_led.py" \
        "$host:/home/pinky/pinky_status_led.py"
    scp -q -o BatchMode=yes -o ConnectTimeout=5 \
        -o ServerAliveInterval=2 -o ServerAliveCountMax=3 \
        "$PROJECT_ROOT/src/vehicle_control/pinky_status_lcd.py" \
        "$host:/home/pinky/pinky_status_lcd.py"
    scp -q -o BatchMode=yes -o ConnectTimeout=5 \
        -o ServerAliveInterval=2 -o ServerAliveCountMax=3 \
        "$PROJECT_ROOT/src/vehicle_control/run_pinky_services.sh" \
        "$host:/home/pinky/run_pinky_services.sh"
    ssh -o BatchMode=yes -o ConnectTimeout=5 "$host" \
        "chmod 755 /home/pinky/pinky_status_led.py /home/pinky/pinky_status_lcd.py /home/pinky/run_pinky_services.sh && nohup setsid -f env ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROBOT_NAMESPACE=$namespace /home/pinky/run_pinky_services.sh $namespace >/home/pinky/pinkk_services_$controller_id.log 2>&1 </dev/null" \
        >"$LOG_DIR/${controller_id}_bringup.log" 2>&1
    REMOTE_PINKY_TARGETS+=("$host|$namespace")
}

if $WITH_PINKY; then
    start_remote_pinky "$PINKY1_HOST" pinkk/vehicle_1 pinky_01
    start_remote_pinky "$PINKY2_HOST" pinkk/vehicle_2 pinky_02
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
if $WITHOUT_CAMERA; then
    echo "Without-camera mode: camera, localization, and video server are disabled."
fi
echo "Press Ctrl+C to stop all services started by this script."

if command -v xdg-open >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
    xdg-open "$URL" >/dev/null 2>&1 || true
fi

wait -n "${CHILD_PIDS[@]}"
echo "ERROR: a service stopped unexpectedly. Check logs in $LOG_DIR" >&2
exit 1
