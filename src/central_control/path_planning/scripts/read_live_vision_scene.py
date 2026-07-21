"""상단 카메라 프로세스가 저장한 최신 start/goal pose를 안전하게 읽는다.

카메라 localization을 먼저 실행한 뒤 별도 터미널에서 실행한다.
오래됐거나 모호한 scene은 경로계획 입력으로 반환하지 않는다.
"""

import argparse
from pathlib import Path
import sys

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
CENTRAL_ROOT = PROJECT_ROOT.parent
sys.path.append(str(PROJECT_ROOT / "src"))

from vision_scene_input import (  # noqa: E402
    VisionSceneUnavailable,
    load_vision_planning_request,
)


DEFAULT_SCENE = PROJECT_ROOT / "output/live_vision_scene.json"
DEFAULT_REGISTRATION = (
    CENTRAL_ROOT
    / "camera_tools/first_map/camera_to_lidar_rigid_registration.npz"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read the latest planning-ready overhead-camera scene."
    )
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument(
        "--registration",
        type=Path,
        default=DEFAULT_REGISTRATION,
    )
    parser.add_argument("--max-age-sec", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        with np.load(args.registration) as registration:
            map_size = (
                int(registration["lidar_width"]),
                int(registration["lidar_height"]),
            )
            resolution_cm = float(registration["resolution"]) * 100.0
        request = load_vision_planning_request(
            args.scene,
            max_age_sec=args.max_age_sec,
            map_size_cells=map_size,
            resolution_cm=resolution_cm,
        )
    except (OSError, KeyError, ValueError, VisionSceneUnavailable) as error:
        print(f"Planning input unavailable: {error}")
        return 1

    print(f"Frame: {request.frame_index}")
    print(f"Selected parking slot: {request.slot_name}")
    print(f"Start rear-axle pose cm: {request.start_pose_cm}")
    print(f"Goal rear-axle pose cm: {request.goal_pose_cm}")
    print(f"Alternative goal pose cm: {request.alternative_goal_pose_cm}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
