#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
STATE_DIR="$PROJECT_ROOT/.runtime/parking_management"
LOCAL_PROCESS_FILE="$STATE_DIR/process_groups"
REMOTE_TARGET_FILE="$STATE_DIR/remote_targets"
PINKY1_HOST="${PINKY1_HOST:-${PINKY_HOST:-pinky@192.168.0.4}}"
PINKY2_HOST="${PINKY2_HOST:-pinky@192.168.0.5}"
STOP_REMOTE=true

if [[ "${1:-}" == "--local-only" ]]; then
    STOP_REMOTE=false
elif [[ $# -gt 0 ]]; then
    echo "Usage: $0 [--local-only]" >&2
    exit 2
fi

declare -A PROCESS_GROUPS=()
MANAGEMENT_FOUND=false

remember_group() {
    local pid="$1"
    local label="$2"
    local pgid
    local current_pgid
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || return 0
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
    [[ "$pgid" =~ ^[1-9][0-9]*$ ]] || return 0
    current_pgid="$(ps -o pgid= -p $$ | tr -d ' ')"
    # PID 1, 사용자 systemd와 이 종료 스크립트의 그룹은 종료하지 않는다.
    if ((pgid <= 1 || pgid == current_pgid)); then
        return 0
    fi
    PROCESS_GROUPS["$pgid"]="$label"
}

if [[ -f "$LOCAL_PROCESS_FILE" ]]; then
    while IFS='|' read -r pid label; do
        command_line="$(ps -o args= -p "$pid" 2>/dev/null || true)"
        case "${label:-}" in
            pinky_ssh_*) expected="run_pinky_services.sh" ;;
            web_video_server) expected="web_video_server" ;;
            rosbridge) expected="rosbridge" ;;
            parking_web) expected="serve_parking_management.py" ;;
            user_web) expected="app_user_parking_coordinate_test.py" ;;
            live_localization) expected="live_localization" ;;
            *) expected="" ;;
        esac
        if [[ -n "$expected" && "$command_line" == *"$expected"* ]]; then
            remember_group "$pid" "$label"
            MANAGEMENT_FOUND=true
        fi
    done <"$LOCAL_PROCESS_FILE"
fi

# 상태 파일 없이 고아가 된 예전 실행도 지원한다. 먼저 8000번 프로세스가 이
# 프로젝트의 관제 웹인지 확인하고, 확인된 경우에만 동반 ROS 포트를 검사한다.
if command -v lsof >/dev/null 2>&1; then
    while IFS= read -r pid; do
        [[ -n "$pid" ]] || continue
        command_line="$(ps -o args= -p "$pid" 2>/dev/null || true)"
        if [[ "$command_line" == *"$PROJECT_ROOT/src/central_control/scripts/serve_parking_management.py"* ]]; then
            remember_group "$pid" "port_8000"
            MANAGEMENT_FOUND=true
        else
            echo "SKIP: port 8000 belongs to another program: $command_line" >&2
        fi
    done < <(lsof -t -nP -iTCP:8000 -sTCP:LISTEN 2>/dev/null | sort -u)

    if $MANAGEMENT_FOUND; then
        for port in 5002 8080 9090; do
            while IFS= read -r pid; do
                [[ -n "$pid" ]] || continue
                command_line="$(ps -o args= -p "$pid" 2>/dev/null || true)"
                if [[
                    ("$port" == 5002 && "$command_line" == *"app_user_parking_coordinate_test.py"*) ||
                    ("$port" == 8080 && "$command_line" == *"web_video_server"*) ||
                    ("$port" == 9090 && "$command_line" == *"rosbridge"*)
                ]]; then
                    remember_group "$pid" "port_${port}"
                else
                    echo "SKIP: port $port belongs to another program: $command_line" >&2
                fi
            done < <(
                lsof -t -nP -iTCP:"$port" -sTCP:LISTEN 2>/dev/null | sort -u
            )
        done
    fi
fi

if ((${#PROCESS_GROUPS[@]} > 0)); then
    for pgid in "${!PROCESS_GROUPS[@]}"; do
        echo "Stopping local ${PROCESS_GROUPS[$pgid]} (process group $pgid)..."
        kill -TERM -- "-$pgid" 2>/dev/null || true
    done
    for _ in {1..30}; do
        any_alive=false
        for pgid in "${!PROCESS_GROUPS[@]}"; do
            if kill -0 -- "-$pgid" 2>/dev/null; then
                any_alive=true
                break
            fi
        done
        $any_alive || break
        sleep 0.1
    done
    for pgid in "${!PROCESS_GROUPS[@]}"; do
        kill -KILL -- "-$pgid" 2>/dev/null || true
    done
else
    echo "No matching local parking-management process is running."
fi

if $STOP_REMOTE; then
    targets=()
    if [[ -f "$REMOTE_TARGET_FILE" ]]; then
        while IFS= read -r target; do
            [[ -n "$target" ]] && targets+=("$target")
        done <"$REMOTE_TARGET_FILE"
    fi
    if ((${#targets[@]} == 0)); then
        targets+=(
            "$PINKY1_HOST|pinkk/vehicle_1"
            "$PINKY2_HOST|pinkk/vehicle_2"
        )
    fi
    for target in "${targets[@]}"; do
        host="${target%%|*}"
        namespace="${target#*|}"
        echo "Stopping remote Pinky $host ($namespace)..."
        ssh -o BatchMode=yes -o ConnectTimeout=3 "$host" \
            "env -u ROS_NAMESPACE ROBOT_NAMESPACE=$namespace /home/pinky/run_pinky_services.sh --stop" \
            >/dev/null 2>&1 || echo "WARN: could not stop $host" >&2
    done
fi

rm -f "$LOCAL_PROCESS_FILE" "$REMOTE_TARGET_FILE"
echo "Parking management cleanup complete."
