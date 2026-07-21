"""상단 카메라 pose에서 검증·저장까지 자동 Hybrid pipeline 회귀 테스트."""

import csv
import json
from pathlib import Path
import sys
import tempfile
import time


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(SCRIPT_DIR))
sys.path.append(str(PROJECT_ROOT / "src"))

from plan_from_live_vision import (  # noqa: E402
    load_planner_stack,
    plan_and_validate,
    save_outputs,
)
from vision_scene_input import VisionPlanningRequest  # noqa: E402


def main() -> int:
    grid_map, planner, profile_config, limits = load_planner_stack()
    # 실제 지도에서 footprint가 통과하는 직선 구간이다. 자동 입력이 클릭 없이
    # planner, smoothing, profile, validator, 파일 저장까지 연결되는지 검사한다.
    request = VisionPlanningRequest(
        frame_index=123,
        observed_at_unix_sec=time.time(),
        slot_name="test_slot",
        start_pose_cm=(90.0, 137.0, 0.0),
        goal_pose_cm=(120.0, 137.0, 0.0),
        alternative_goal_pose_cm=(120.0, 137.0, 3.141592653589793),
    )
    (
        selected_candidate,
        adjusted_start,
        adjusted_goal,
        result,
        trajectory,
        validation,
    ) = plan_and_validate(
        planner,
        profile_config,
        limits,
        request.start_pose_cm,
        (
            ("primary goal", request.goal_pose_cm),
            ("alternative goal", request.alternative_goal_pose_cm),
        ),
    )
    assert validation.valid
    assert trajectory
    assert selected_candidate == "primary goal"
    assert result.smoothing_stats is not None
    assert result.smoothing_stats.accepted

    with tempfile.TemporaryDirectory() as directory:
        output_dir = Path(directory)
        save_outputs(
            request,
            selected_candidate,
            adjusted_start,
            adjusted_goal,
            result,
            trajectory,
            validation,
            grid_map.resolution_cm,
            profile_config,
            output_dir=output_dir,
        )
        world_path = output_dir / "live_hybrid_path_world_cm.csv"
        camera_path = output_dir / "live_hybrid_path_camera_bev.csv"
        json_path = output_dir / "live_hybrid_path_world_cm.json"
        assert world_path.exists() and camera_path.exists() and json_path.exists()
        with world_path.open(encoding="utf-8") as file:
            rows = list(csv.DictReader(file))
        with json_path.open(encoding="utf-8") as file:
            payload = json.load(file)
        assert len(rows) == len(trajectory)
        assert payload["source"]["frame_index"] == 123
        assert payload["source"]["parking_slot"] == "test_slot"
        assert len(payload["path"]) == len(trajectory)

    print("Live vision to Hybrid A* pipeline regression passed")
    print(f"Trajectory points: {len(trajectory)}")
    print(f"Goal connection: {result.message}")
    print(f"Total cost: {result.total_cost:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
