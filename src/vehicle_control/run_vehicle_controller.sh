#!/usr/bin/env bash
set -euo pipefail

if (($# != 1)); then
    echo "Usage: $0 vehicle_1|vehicle_2" >&2
    exit 2
fi

VEHICLE_ID="$1"
case "$VEHICLE_ID" in
    vehicle_1|vehicle_2) ;;
    *)
        echo "ERROR: unknown vehicle ID: $VEHICLE_ID" >&2
        exit 2
        ;;
esac

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VEHICLE_NAMESPACE="/pinkk/$VEHICLE_ID"
FUSED_CONFIG="$PROJECT_ROOT/src/vehicle_control/config/localization/fused_pose.yaml"
MPC_CONFIG="$PROJECT_ROOT/src/vehicle_control/config/mpc/mpc.yaml"
MAP_IMAGE="$PROJECT_ROOT/src/central_control/camera_tools/first_map/my_test_map0710.png"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"
CHILD_PIDS=()

set +u
source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

cleanup() {
    trap - HUP INT TERM EXIT
    echo
    echo "Stopping vehicle controller: $VEHICLE_ID"
    # ros2 CLI wrapper만 종료하면 실제 Python node가 고아 프로세스로 남는다.
    # 각 node를 별도 session으로 시작하고 process group 전체에 TERM을 보낸다.
    for pid in "${CHILD_PIDS[@]}"; do
        kill -TERM -- "-$pid" 2>/dev/null || true
    done

    # 정상 shutdown 동안 MPC가 마지막 0속도 명령을 발행할 시간을 준다.
    for _ in {1..30}; do
        any_alive=false
        for pid in "${CHILD_PIDS[@]}"; do
            if kill -0 -- "-$pid" 2>/dev/null; then
                any_alive=true
                break
            fi
        done
        "$any_alive" || break
        sleep 0.1
    done

    # DDS 또는 ros2 wrapper가 종료를 막아도 스크립트가 wait에서 무한히
    # 멈추지 않도록 남은 process group만 최종 정리한다.
    for pid in "${CHILD_PIDS[@]}"; do
        kill -KILL -- "-$pid" 2>/dev/null || true
    done
    wait "${CHILD_PIDS[@]}" 2>/dev/null || true
}
trap cleanup HUP INT TERM EXIT

echo "Starting vehicle controller: $VEHICLE_ID"
echo "ROS namespace: $VEHICLE_NAMESPACE"

setsid ros2 run pinkk fused_pose_estimator \
    --ros-args \
    -r "__ns:=$VEHICLE_NAMESPACE" \
    --params-file "$FUSED_CONFIG" \
    -p "map_image_path:=$MAP_IMAGE" &
CHILD_PIDS+=("$!")

setsid ros2 run pinkk mpc_path_follower \
    --ros-args \
    -r "__ns:=$VEHICLE_NAMESPACE" \
    --params-file "$MPC_CONFIG" \
    -p "tuning_file:=$MPC_CONFIG" &
CHILD_PIDS+=("$!")

set +e
wait -n "${CHILD_PIDS[@]}"
exit_code=$?
set -e
if ((exit_code == 130 || exit_code == 143)); then
    exit 0
fi
echo "ERROR: a $VEHICLE_ID control node exited (code=$exit_code)" >&2
if ((exit_code == 0)); then
    exit 1
fi
exit "$exit_code"
