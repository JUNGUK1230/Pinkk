"""ROS 없이 trajectory publisher의 JSON 변환 규약을 회귀 테스트한다."""

import json
from pathlib import Path
import sys
import tempfile


SCRIPT_DIR = Path(__file__).resolve().parent
CENTRAL_ROOT = SCRIPT_DIR.parents[1]
sys.path.append(str(CENTRAL_ROOT))

from overhead_vision.path_planning.path_publisher import (  # noqa: E402
    TRAJECTORY_FIELDS,
    load_validated_trajectory,
    trajectory_matrix,
)
from overhead_vision.path_planning.vehicle_pose_publisher import (  # noqa: E402
    load_current_vehicle_pose,
)


def main() -> int:
    payload = {
        "planner": "hybrid_astar",
        "validation_metrics": {"point_count": 2},
        "path": [
            {
                "x_cm": 12.0,
                "y_cm": 34.0,
                "yaw_rad": 0.5,
                "direction": 1,
                "target_speed_mps": 0.03,
                "steer_deg": 10.0,
                "stop_required": 0,
            },
            {
                "x_cm": 15.0,
                "y_cm": 36.0,
                "yaw_rad": 0.7,
                "direction": -1,
                "target_speed_mps": -0.02,
                "steer_deg": -10.0,
                "stop_required": 1,
            },
        ],
    }
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "trajectory.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        loaded = load_validated_trajectory(path)
        matrix = trajectory_matrix(loaded)
        assert len(matrix) == 2 * len(TRAJECTORY_FIELDS)
        assert matrix[:4] == [0.12, 0.34, 0.5, 1.0]
        assert matrix[len(TRAJECTORY_FIELDS) + 3] == -1.0

        payload["control_ready"] = False
        path.write_text(json.dumps(payload), encoding="utf-8")
        try:
            load_validated_trajectory(path)
        except ValueError as error:
            assert "control_ready" in str(error)
        else:
            raise AssertionError("control_ready=false must be rejected")

        scene_path = Path(directory) / "scene.json"
        scene_path.write_text(
            json.dumps(
                {
                    "frame_index": 37,
                    "observed_at_unix_sec": 100.0,
                    "planning_ready": True,
                    "status": "planning input ready",
                    "vehicle": {
                        "center_cm": [45.0, 72.0],
                        "yaw_rad": 0.25,
                    },
                }
            ),
            encoding="utf-8",
        )
        pose = load_current_vehicle_pose(
            scene_path,
            max_age_sec=0.5,
            now_unix_sec=100.2,
        )
        assert pose == (0.45, 0.72, 0.25, 37)
        try:
            load_current_vehicle_pose(
                scene_path,
                max_age_sec=0.5,
                now_unix_sec=101.0,
            )
        except ValueError as error:
            assert "stale" in str(error)
        else:
            raise AssertionError("stale vehicle pose must be rejected")

    print("Trajectory publisher conversion regression passed")
    print("Vehicle pose conversion and stale-scene regression passed")
    print(f"Fields: {', '.join(TRAJECTORY_FIELDS)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
