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
    _nearby_goal_poses,
    adjust_pose,
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
    assert selected_candidate in ("primary goal", "alternative goal")
    assert result.smoothing_stats is not None
    assert result.smoothing_stats.accepted

    # C1 nominal goal 주위의 3 cm 후보가 명목 pose부터 square-ring 순서로
    # 생성되는지 검사한다. 이전 P5 planning 회귀는 3 cm 안전마진에서 해당
    # 구간 자체가 좁아졌으므로 실제 고정 목표인 C1 사례로 대체한다.
    nominal_c1_goal = adjust_pose(
        planner,
        (162.20323667859284, 123.87150109209493, 0.689413868664666),
        "primary goal",
    )
    nearby_goals = _nearby_goal_poses(planner, nominal_c1_goal, "primary goal")
    assert nearby_goals[0][0] == "primary goal"
    assert any("offset" in label for label, _ in nearby_goals[1:])

    # 최신 현장 수동 heading 좌표에서 inflated 탐색 경로의 clearance를
    # 확보해 smoothing과 최종 steering-rate 검증을 통과해야 한다.
    c1_result = plan_and_validate(
        planner,
        profile_config,
        limits,
        (14.330144587043513, 70.35593523821866, 0.8453690022091003),
        (
            (
                "primary goal",
                (162.20323667859284, 123.87150109209493, 0.689413868664666),
            ),
            (
                "alternative goal",
                (168.3761884936616, 128.96018126369538, -2.4521787849251275),
            ),
        ),
        planning_budget_sec=0.0,
    )
    assert c1_result[0] == "alternative goal"
    assert c1_result[5].valid
    assert c1_result[5].metrics.max_abs_steer_deg <= 30.0

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
            bev_image_path=(
                PROJECT_ROOT.parent
                / "camera_tools/first_map/camera_bev.png"
            ),
        )
        world_path = output_dir / "live_hybrid_path_world_cm.csv"
        camera_path = output_dir / "live_hybrid_path_camera_bev.csv"
        json_path = output_dir / "live_hybrid_path_world_cm.json"
        overlay_path = output_dir / "live_hybrid_path_on_camera_bev.png"
        assert world_path.exists() and camera_path.exists() and json_path.exists()
        assert overlay_path.exists()
        with world_path.open(encoding="utf-8") as file:
            rows = list(csv.DictReader(file))
        with json_path.open(encoding="utf-8") as file:
            payload = json.load(file)
        assert len(rows) == len(trajectory)
        assert payload["source"]["frame_index"] == 123
        assert payload["source"]["parking_slot"] == "test_slot"
        assert len(payload["path"]) == len(trajectory)
        assert payload["visualization"]["out_of_bounds_path_points"] == 0

    print("Live vision to Hybrid A* pipeline regression passed")
    print(f"Trajectory points: {len(trajectory)}")
    print(f"Goal connection: {result.message}")
    print(f"Total cost: {result.total_cost:.3f}")
    print(f"Nearby-goal generation regression: {len(nearby_goals)} poses")
    print(
        "C1 long-range regression: "
        f"{c1_result[0]}, points={len(c1_result[4])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
