"""상단 카메라 최신 scene을 Hybrid A* start/goal pose로 읽는다."""

from dataclasses import dataclass
import json
import math
from pathlib import Path
import time


Pose = tuple[float, float, float]


class VisionSceneUnavailable(RuntimeError):
    """검출 누락·모호성·stale 데이터로 경로계획을 시작할 수 없음."""


@dataclass(frozen=True)
class VisionPlanningRequest:
    frame_index: int
    observed_at_unix_sec: float
    slot_name: str
    start_pose_cm: Pose
    goal_pose_cm: Pose
    alternative_goal_pose_cm: Pose


def load_vision_planning_request(
    scene_path: str | Path,
    max_age_sec: float = 0.5,
    map_size_cells: tuple[int, int] | None = None,
    resolution_cm: float = 1.0,
    now_unix_sec: float | None = None,
) -> VisionPlanningRequest:
    """최신 planning_ready scene만 반환하고 나머지는 fail-closed로 거부한다."""
    if max_age_sec <= 0.0 or resolution_cm <= 0.0:
        raise ValueError("max_age_sec and resolution_cm must be positive")
    path = Path(scene_path)
    if not path.exists():
        raise VisionSceneUnavailable(f"vision scene file not found: {path}")
    try:
        with path.open(encoding="utf-8") as file:
            payload = json.load(file)
    except (json.JSONDecodeError, OSError) as error:
        raise VisionSceneUnavailable(f"could not read vision scene: {error}") from error

    if payload.get("planning_ready") is not True:
        raise VisionSceneUnavailable(
            f"vision scene is not planning-ready: {payload.get('status', 'unknown')}"
        )
    request = payload.get("planning_request")
    if not isinstance(request, dict):
        raise VisionSceneUnavailable("planning_request is missing")
    try:
        frame_index = int(payload["frame_index"])
        observed_at = float(payload["observed_at_unix_sec"])
        slot_name = str(request["slot_name"])
        start = _read_pose(request["start_pose_cm"], "start_pose_cm")
        goal = _read_pose(request["goal_pose_cm"], "goal_pose_cm")
        alternative = _read_pose(
            request["alternative_goal_pose_cm"],
            "alternative_goal_pose_cm",
        )
    except (KeyError, TypeError, ValueError) as error:
        raise VisionSceneUnavailable(f"invalid planning request: {error}") from error

    now = time.time() if now_unix_sec is None else now_unix_sec
    age_sec = now - observed_at
    if age_sec < -0.1 or age_sec > max_age_sec:
        raise VisionSceneUnavailable(
            f"vision scene is stale or time-invalid: age={age_sec:.3f} sec"
        )
    if map_size_cells is not None:
        width, height = map_size_cells
        if width <= 0 or height <= 0:
            raise ValueError("map dimensions must be positive")
        for label, pose in (("start", start), ("goal", goal), ("alternative", alternative)):
            if not (
                0.0 <= pose[0] < width * resolution_cm
                and 0.0 <= pose[1] < height * resolution_cm
            ):
                raise VisionSceneUnavailable(
                    f"{label} pose is outside the planning map: {pose}"
                )
    return VisionPlanningRequest(
        frame_index,
        observed_at,
        slot_name,
        start,
        goal,
        alternative,
    )


def _read_pose(value: object, label: str) -> Pose:
    if not isinstance(value, dict):
        raise TypeError(f"{label} must be an object")
    pose = (
        float(value["x_cm"]),
        float(value["y_cm"]),
        float(value["yaw_rad"]),
    )
    if not all(math.isfinite(component) for component in pose):
        raise ValueError(f"{label} must contain finite values")
    return pose
