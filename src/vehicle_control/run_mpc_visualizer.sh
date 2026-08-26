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
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-36}"

set +u
source /opt/ros/jazzy/setup.bash
source "$PROJECT_ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/pinkk-matplotlib}"

cd "$PROJECT_ROOT"
echo "Starting do-mpc visualizer: $VEHICLE_ID"
echo "ROS namespace: $VEHICLE_NAMESPACE"
exec "$PROJECT_ROOT/.venv/bin/python" -m src.vehicle_control.mpc_visualizer \
    --ros-args \
    -r "__ns:=$VEHICLE_NAMESPACE"
