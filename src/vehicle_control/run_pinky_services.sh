#!/usr/bin/env bash
set -euo pipefail

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROBOT_NAMESPACE="${ROBOT_NAMESPACE:-pinkk/vehicle_1}"
ROBOT_NAMESPACE="${ROBOT_NAMESPACE#/}"
PINKY_ROOT="${PINKY_ROOT:-$HOME/pinky_pro}"
PINKY_CTRL_ROOT="${PINKY_CTRL_ROOT:-$HOME/pinky_ctrl}"
# base: 모터·LiDAR·배터리만 실행하는 기존 pinky_bringup을 사용한다.
# ctrl: lane_controller와 EKF까지 포함된 pinky_ctrl bringup을 명시적으로 사용한다.
# 중앙 MPC와 cmd_vel 충돌을 막기 위해 기본값은 base다.
PINKY_BRINGUP_MODE="${PINKY_BRINGUP_MODE:-base}"
LED_NODE="${LED_NODE:-$HOME/pinky_status_led.py}"
LCD_NODE="${LCD_NODE:-$HOME/pinky_status_lcd.py}"
PINKY_LCD_FONT="${PINKY_LCD_FONT:-$HOME/pinky_lcd/example/MaruBuri-Bold.ttf}"
CHILD_PIDS=()

set +u
source /opt/ros/jazzy/setup.bash
source "$PINKY_ROOT/install/setup.bash"
USE_CTRL_BRINGUP=false
case "$PINKY_BRINGUP_MODE" in
    base) ;;
    ctrl)
        if [[ ! -f "$PINKY_CTRL_ROOT/install/setup.bash" ]]; then
            echo "ERROR: pinky_ctrl is not installed: $PINKY_CTRL_ROOT" >&2
            exit 2
        fi
        source "$PINKY_CTRL_ROOT/install/setup.bash"
        USE_CTRL_BRINGUP=true
        ;;
    *)
        echo "ERROR: PINKY_BRINGUP_MODE must be base or ctrl" >&2
        exit 2
        ;;
esac
set -u
export ROS_DOMAIN_ID
export ROS_AUTOMATIC_DISCOVERY_RANGE
export ROS_STATIC_PEERS
export RMW_IMPLEMENTATION
export ROS_LOCALHOST_ONLY=0
# root 권한 LED 노드와 일반 사용자 ROS 노드 사이의 Fast DDS shared-memory
# 권한 충돌을 피하고 동일한 UDP transport로 통신한다.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROBOT_NAMESPACE
export PINKY_LCD_FONT
# SSH 로그인 환경이나 이전 bringup이 남긴 전역 namespace가 push-namespace와
# 겹치면 /pinkk/vehicle_2/pinkk/vehicle_2처럼 중복된다. 이 스크립트에서는
# launch 인자로만 namespace를 지정한다.
unset ROS_NAMESPACE

PID_FILE="/tmp/pinkk_services_${ROBOT_NAMESPACE//\//_}.pid"
SERVICE_PATTERNS=(
    'ros2 launch pinky_bringup bringup_robot.launch.xml'
    'ros2 launch pinky_ctrl bringup_imu.launch.py'
    '/pinky_bringup/lib/pinky_bringup/'
    '/pinky_ctrl/lib/'
    '/sllidar_ros2/lib/sllidar_ros2/sllidar_node'
    '/pinky_bringup/battery_publisher'
    "python3 $LED_NODE"
    "python3 $LCD_NODE"
)

stop_existing_services() {
    local supervisor_pid
    supervisor_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [[ "$supervisor_pid" =~ ^[1-9][0-9]*$ ]] \
        && kill -0 "$supervisor_pid" 2>/dev/null; then
        kill -TERM "$supervisor_pid" 2>/dev/null || true
        for _ in {1..50}; do
            kill -0 "$supervisor_pid" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL "$supervisor_pid" 2>/dev/null || true
    fi
    for pattern in "${SERVICE_PATTERNS[@]}"; do
        pkill -TERM -f "$pattern" 2>/dev/null || true
    done
    sleep 1
    for pattern in "${SERVICE_PATTERNS[@]}"; do
        pkill -KILL -f "$pattern" 2>/dev/null || true
    done
    python3 "$LCD_NODE" --clear-only >/dev/null 2>&1 || true
    rm -f "$PID_FILE"
}

if [[ "${1:-}" == "--stop" ]]; then
    stop_existing_services
    echo "Pinky bringup, LED, and LCD services stopped: $ROBOT_NAMESPACE"
    exit 0
fi

if [[ -f "$PID_FILE" ]]; then
    previous_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [[ "$previous_pid" =~ ^[1-9][0-9]*$ ]] \
        && kill -0 "$previous_pid" 2>/dev/null; then
        echo "ERROR: Pinky service supervisor is already running: $previous_pid" >&2
        exit 1
    fi
fi
echo "$$" >"$PID_FILE"

cleanup() {
    trap - HUP INT TERM EXIT
    echo "Stopping Pinky bringup, LED, and LCD services..."
    for pid in "${CHILD_PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    for _ in {1..30}; do
        any_alive=false
        for pid in "${CHILD_PIDS[@]}"; do
            if kill -0 -- "-$pid" 2>/dev/null; then
                any_alive=true
                break
            fi
        done
        $any_alive || break
        sleep 0.1
    done
    for pid in "${CHILD_PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
    wait "${CHILD_PIDS[@]}" 2>/dev/null || true
    python3 "$LCD_NODE" --clear-only >/dev/null 2>&1 || true
    current_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
    if [[ "$current_pid" == "$$" ]]; then
        rm -f "$PID_FILE"
    fi
}

trap cleanup HUP INT TERM EXIT

VEHICLE_NUMBER="${ROBOT_NAMESPACE##*_}"
VEHICLE_ID="vehicle_$VEHICLE_NUMBER"
CONTROLLER_ID="$(printf 'pinky_%02d' "$VEHICLE_NUMBER")"
HARDWARE_SERIAL="$(printf 'PINKY-%03d' "$VEHICLE_NUMBER")"
# 이전 supervisor가 비정상 종료해도 하드웨어 프로세스가 새 실행으로 넘어오지
# 않도록, 이 차량 전용 Raspberry Pi의 bringup/상태 노드를 먼저 정리한다.
for pattern in "${SERVICE_PATTERNS[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
done
sleep 2
for pattern in "${SERVICE_PATTERNS[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
done

if $USE_CTRL_BRINGUP; then
    setsid ros2 launch pinky_ctrl bringup_imu.launch.py \
        namespace:="$ROBOT_NAMESPACE" \
        vehicle_id:="$VEHICLE_ID" \
        controller_id:="$CONTROLLER_ID" \
        hardware_serial:="$HARDWARE_SERIAL" &
else
    setsid ros2 launch pinky_bringup bringup_robot.launch.xml \
        namespace:="$ROBOT_NAMESPACE" \
        vehicle_id:="$VEHICLE_ID" \
        controller_id:="$CONTROLLER_ID" \
        hardware_serial:="$HARDWARE_SERIAL" &
fi
CHILD_PIDS+=("$!")

setsid sudo -E env \
    PYTHONPATH="${PYTHONPATH:-}" \
    LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}" \
    AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}" \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
    python3 "$LED_NODE" --ros-args \
    -r __ns:="/$ROBOT_NAMESPACE" &
CHILD_PIDS+=("$!")

setsid python3 "$LCD_NODE" --ros-args \
    -r __ns:="/$ROBOT_NAMESPACE" &
CHILD_PIDS+=("$!")

sleep 2
for pid in "${CHILD_PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "ERROR: a Pinky service exited during startup." >&2
        exit 1
    fi
done

echo "Pinky bringup, status LED, and status LCD services are running."
echo "ROS discovery: range=$ROS_AUTOMATIC_DISCOVERY_RANGE peers=${ROS_STATIC_PEERS:-none} rmw=$RMW_IMPLEMENTATION"
wait -n "${CHILD_PIDS[@]}"
echo "ERROR: a Pinky service stopped unexpectedly." >&2
exit 1
