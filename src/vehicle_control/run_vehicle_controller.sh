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
CHILD_PIDS=()

set +u
source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
set -u

cleanup() {
    trap - INT TERM EXIT
    for pid in "${CHILD_PIDS[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    wait "${CHILD_PIDS[@]}" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "Starting vehicle controller: $VEHICLE_ID"
echo "ROS namespace: $VEHICLE_NAMESPACE"

ros2 run pinkk fused_pose_estimator \
    --ros-args \
    -r "__ns:=$VEHICLE_NAMESPACE" \
    --params-file "$FUSED_CONFIG" &
CHILD_PIDS+=("$!")

ros2 run pinkk mpc_path_follower \
    --ros-args \
    -r "__ns:=$VEHICLE_NAMESPACE" \
    --params-file "$MPC_CONFIG" &
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
