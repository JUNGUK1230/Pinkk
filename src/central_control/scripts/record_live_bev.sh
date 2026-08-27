#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
RECORDING_DIR="$PROJECT_ROOT/src/central_control/camera_tools/first_map/bev_recordings"
VIDEO_PORT="${PINKK_VIDEO_PORT:-8080}"
VIDEO_TOPIC="${PINKK_VIDEO_TOPIC:-/pinkk/camera_bev/image}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_PATH="${1:-$RECORDING_DIR/bev_live_$TIMESTAMP.mkv}"
STREAM_URL="http://127.0.0.1:$VIDEO_PORT/stream?topic=$VIDEO_TOPIC"

if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "ERROR: ffmpeg is not installed." >&2
    exit 1
fi

mkdir -p "$(dirname "$OUTPUT_PATH")"

echo "Recording pure BEV stream: $VIDEO_TOPIC"
echo "Saved video: $OUTPUT_PATH"
echo "Press Ctrl+C to stop and finalize the video."

ffmpeg \
    -hide_banner \
    -loglevel warning \
    -fflags +genpts \
    -i "$STREAM_URL" \
    -an \
    -c:v copy \
    -n \
    "$OUTPUT_PATH"
