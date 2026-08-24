#!/usr/bin/env bash

# Pinky Wi-Fi에서는 인터넷이 없어도 실행할 수 있다. 수집 결과는 로컬 파일에
# 남으므로, 실행 후 인터넷 Wi-Fi로 돌아와 같은 파일을 확인하면 된다.
set -uo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
REPORT_DIR="$PROJECT_ROOT/.runtime/parking_management"
REPORT_PATH="$REPORT_DIR/pinky_diagnostics_latest.log"

mkdir -p "$REPORT_DIR"
exec > >(tee "$REPORT_PATH") 2>&1

section() {
    echo
    echo "===== $1 ====="
}

run_limited() {
    local seconds="$1"
    shift
    timeout "${seconds}s" "$@" || echo "[exit=$?] $*"
}

section "COLLECTION INFO"
date --iso-8601=seconds
echo "report=$REPORT_PATH"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "FASTDDS_BUILTIN_TRANSPORTS=UDPv4"

section "CENTRAL PC NETWORK"
ip -brief -4 address || true
ip route || true
nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device status 2>/dev/null || true

section "PINKY REACHABILITY"
for address in 192.168.0.4 192.168.0.5; do
    echo "--- $address ---"
    run_limited 4 ping -c 2 -W 1 "$address"
done

set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$PROJECT_ROOT/install/setup.bash" ]]; then
    source "$PROJECT_ROOT/install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

section "CENTRAL ROS TOPICS"
run_limited 10 ros2 topic list
for vehicle in vehicle_1 vehicle_2; do
    for suffix in scan battery/percent battery/voltage; do
        topic="/pinkk/$vehicle/$suffix"
        echo "--- $topic ---"
        run_limited 8 ros2 topic info -v "$topic"
    done
done

section "CENTRAL ROS SAMPLE RATES"
for vehicle in vehicle_1 vehicle_2; do
    for suffix in scan battery/percent; do
        topic="/pinkk/$vehicle/$suffix"
        echo "--- $topic ---"
        run_limited 6 ros2 topic hz "$topic" --window 5
    done
done

collect_remote() {
    local host="$1"
    local vehicle="$2"
    local controller="$3"
    section "REMOTE $controller ($host, $vehicle)"
    run_limited 12 ssh \
        -o BatchMode=yes \
        -o ConnectTimeout=4 \
        -o ServerAliveInterval=2 \
        -o ServerAliveCountMax=2 \
        "$host" \
        "echo HOST=\$(hostname); \
         echo --- PROCESSES ---; \
         pgrep -af 'run_pinky_services|bringup_robot|sllidar|battery_publisher' || true; \
         echo --- SERIAL DEVICES ---; \
         ls -l /dev/ttyS0 /dev/ttyUSB0 /dev/ttyUSB1 2>&1 || true; \
         echo --- SERVICE LOG ---; \
         tail -n 180 /home/pinky/pinkk_services_${controller}.log 2>&1 || true"
}

collect_remote pinky@192.168.0.4 vehicle_1 pinky_01
collect_remote pinky@192.168.0.5 vehicle_2 pinky_02

section "LOCALIZATION IDENTITY LOG"
if [[ -f "$REPORT_DIR/localization.log" ]]; then
    grep -En \
        "LiDAR|association|identity|waiting_scan|Selected path target|Automatic ROS|ERROR|Traceback" \
        "$REPORT_DIR/localization.log" | tail -n 240 || true
else
    echo "localization.log not found"
fi

section "DONE"
echo "Saved diagnostic report: $REPORT_PATH"
