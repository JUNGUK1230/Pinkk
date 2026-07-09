#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/../../.."
python3 -m src.central_control.overhead_vision.camera.camera_capture
