#!/usr/bin/env bash
set -euo pipefail

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
ROBOT_NAMESPACE="${ROBOT_NAMESPACE:-pinkk/vehicle_1}"
ROBOT_NAMESPACE="${ROBOT_NAMESPACE#/}"
PINKY_ROOT="${PINKY_ROOT:-$HOME/pinky_pro}"
PINKY_CTRL_ROOT="${PINKY_CTRL_ROOT:-$HOME/pinky_ctrl}"
LED_NODE="${LED_NODE:-$HOME/pinky_status_led.py}"
LCD_NODE="${LCD_NODE:-$HOME/pinky_status_lcd.py}"
PINKY_LCD_FONT="${PINKY_LCD_FONT:-$HOME/pinky_lcd/example/MaruBuri-Bold.ttf}"
CHILD_PIDS=()

set +u
source /opt/ros/jazzy/setup.bash
source "$PINKY_ROOT/install/setup.bash"
USE_CTRL_BRINGUP=false
if [[ -f "$PINKY_CTRL_ROOT/install/setup.bash" ]]; then
    source "$PINKY_CTRL_ROOT/install/setup.bash"
    USE_CTRL_BRINGUP=true
fi
set -u
export ROS_DOMAIN_ID
# root 권한 LED 노드와 일반 사용자 ROS 노드 사이의 Fast DDS shared-memory
# 권한 충돌을 피하고 동일한 UDP transport로 통신한다.
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROBOT_NAMESPACE
export PINKY_LCD_FONT

cleanup() {
    trap - HUP INT TERM EXIT
    for pid in "${CHILD_PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done
    wait "${CHILD_PIDS[@]}" 2>/dev/null || true
    for pid in "${CHILD_PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
}

trap cleanup HUP INT TERM EXIT

EXPECTED_BATTERY_TOPIC="/$ROBOT_NAMESPACE/battery/percent"
VEHICLE_NUMBER="${ROBOT_NAMESPACE##*_}"
VEHICLE_ID="vehicle_$VEHICLE_NUMBER"
CONTROLLER_ID="$(printf 'pinky_%02d' "$VEHICLE_NUMBER")"
HARDWARE_SERIAL="$(printf 'PINKY-%03d' "$VEHICLE_NUMBER")"
if $USE_CTRL_BRINGUP; then
    BRINGUP_PATTERN='ros2 launch pinky_ctrl bringup_imu.launch.py'
    LEGACY_BRINGUP_PATTERN='ros2 launch pinky_bringup bringup_robot.launch.xml'
else
    BRINGUP_PATTERN='ros2 launch pinky_bringup bringup_robot.launch.xml'
    LEGACY_BRINGUP_PATTERN=''
fi

# 예전 launch는 namespace 인자를 선언하지 않아 /battery/*를 발행한다.
# 동시에 남아 있으면 센서와 모터 장치를 중복 점유하므로 먼저 종료한다.
legacy_bringup_pids=()
if [[ -n "$LEGACY_BRINGUP_PATTERN" ]]; then
    mapfile -t legacy_bringup_pids < <(
        pgrep -f "$LEGACY_BRINGUP_PATTERN" || true
    )
fi
if ((${#legacy_bringup_pids[@]} > 0)); then
    echo "Stopping legacy non-namespaced Pinky bringup."
    kill -TERM "${legacy_bringup_pids[@]}" 2>/dev/null || true
    sleep 2
    mapfile -t legacy_bringup_pids < <(
        pgrep -f "$LEGACY_BRINGUP_PATTERN" || true
    )
    ((${#legacy_bringup_pids[@]} == 0)) \
        || kill -KILL "${legacy_bringup_pids[@]}" 2>/dev/null \
        || true
    pkill -TERM -f '/pinky_bringup/lib/pinky_bringup/' \
        2>/dev/null || true
    pkill -TERM -f '/sllidar_ros2/lib/sllidar_ros2/sllidar_node' \
        2>/dev/null || true
fi

if pgrep -f "$BRINGUP_PATTERN" >/dev/null; then
    publisher_count="$(
        ros2 topic info "$EXPECTED_BATTERY_TOPIC" 2>/dev/null \
            | awk '/Publisher count:/ {print $3}' \
            || true
    )"
    if [[ "${publisher_count:-0}" =~ ^[1-9][0-9]*$ ]]; then
        echo "Namespaced Pinky bringup is already running; reusing it."
    else
        echo "Stopping a stale or incorrectly namespaced Pinky bringup."
        mapfile -t stale_bringup_pids < <(pgrep -f "$BRINGUP_PATTERN" || true)
        ((${#stale_bringup_pids[@]} == 0)) \
            || kill -TERM "${stale_bringup_pids[@]}" 2>/dev/null \
            || true
        for _ in {1..50}; do
            pgrep -f "$BRINGUP_PATTERN" >/dev/null || break
            sleep 0.1
        done
        mapfile -t stale_bringup_pids < <(pgrep -f "$BRINGUP_PATTERN" || true)
        ((${#stale_bringup_pids[@]} == 0)) \
            || kill -KILL "${stale_bringup_pids[@]}" 2>/dev/null \
            || true
    fi
fi

if ! pgrep -f "$BRINGUP_PATTERN" >/dev/null; then
    if $USE_CTRL_BRINGUP; then
        setsid ros2 launch pinky_ctrl bringup_imu.launch.py \
            namespace:="$ROBOT_NAMESPACE" \
            vehicle_id:="$VEHICLE_ID" \
            controller_id:="$CONTROLLER_ID" \
            hardware_serial:="$HARDWARE_SERIAL" &
    else
        setsid ros2 launch pinky_bringup bringup_robot.launch.xml \
            namespace:="$ROBOT_NAMESPACE" &
    fi
    CHILD_PIDS+=("$!")
fi

if pgrep -f "python3 $LED_NODE" >/dev/null; then
    echo "Pinky status LED service is already running; reusing it."
else
    setsid sudo -E env \
        PYTHONPATH="${PYTHONPATH:-}" \
        LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}" \
        AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}" \
        FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
        python3 "$LED_NODE" --ros-args \
        -r __ns:="/$ROBOT_NAMESPACE" &
    CHILD_PIDS+=("$!")
fi

if pgrep -f "python3 $LCD_NODE" >/dev/null; then
    echo "Pinky status LCD service is already running; reusing it."
else
    setsid python3 "$LCD_NODE" --ros-args \
        -r __ns:="/$ROBOT_NAMESPACE" &
    CHILD_PIDS+=("$!")
fi

if ((${#CHILD_PIDS[@]} == 0)); then
    echo "Pinky bringup, status LED, and status LCD services are already running."
    while true; do
        sleep 60
    done
fi

sleep 2
for pid in "${CHILD_PIDS[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "ERROR: a Pinky service exited during startup." >&2
        exit 1
    fi
done

echo "Pinky bringup, status LED, and status LCD services are running."
wait -n "${CHILD_PIDS[@]}"
echo "ERROR: a Pinky service stopped unexpectedly." >&2
exit 1
