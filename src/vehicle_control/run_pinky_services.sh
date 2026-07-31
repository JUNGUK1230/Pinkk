#!/usr/bin/env bash
set -euo pipefail

ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
PINKY_ROOT="${PINKY_ROOT:-$HOME/pinky_pro}"
LCD_NODE="${LCD_NODE:-$HOME/pinky_emergency_lcd.py}"
PINKY_LCD_FONT="${PINKY_LCD_FONT:-$HOME/pinky_lcd/example/MaruBuri-Bold.ttf}"
CHILD_PIDS=()

set +u
source /opt/ros/jazzy/setup.bash
source "$PINKY_ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID
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

if pgrep -f 'ros2 launch pinky_bringup bringup_robot.launch.xml' >/dev/null; then
    echo "Pinky bringup is already running; reusing it."
else
    setsid ros2 launch pinky_bringup bringup_robot.launch.xml namespace:=pinky1 &
    CHILD_PIDS+=("$!")
fi

if pgrep -f "python3 $LCD_NODE" >/dev/null; then
    echo "Pinky emergency LCD service is already running; reusing it."
else
    setsid python3 "$LCD_NODE" &
    CHILD_PIDS+=("$!")
fi

if ((${#CHILD_PIDS[@]} == 0)); then
    echo "Pinky bringup and emergency LCD service are already running."
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

echo "Pinky bringup and emergency LCD service are running."
wait -n "${CHILD_PIDS[@]}"
echo "ERROR: a Pinky service stopped unexpectedly." >&2
exit 1
