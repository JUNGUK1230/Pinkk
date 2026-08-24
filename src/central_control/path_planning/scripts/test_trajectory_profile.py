"""Regression test for curvature-aware Hybrid A* trajectory speeds."""

import math
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
sys.path.append(str(PROJECT_ROOT / "src"))

from hybrid_astar_planner import HybridState  # noqa: E402
from trajectory_profile import build_trajectory_profile  # noqa: E402


def main() -> int:
    """Verify curve slowdown, signed reverse speed, and mandatory stops."""
    path: list[HybridState] = []
    for index in range(120):
        direction = 1 if index < 80 else -1
        steer_deg = 0.0 if index < 40 or index >= 80 else 30.0
        path.append(
            HybridState(
                x_cm=index * 0.5,
                y_cm=20.0,
                yaw_rad=0.0,
                direction=direction,
                steer_rad=math.radians(steer_deg),
            )
        )

    trajectory = build_trajectory_profile(
        path,
        wheelbase_cm=8.0,
        max_steer_rad=math.radians(30.0),
    )
    assert len(trajectory) == len(path)
    assert trajectory[0].stop_required and trajectory[0].target_speed_mps == 0.0
    assert trajectory[-1].stop_required and trajectory[-1].target_speed_mps == 0.0
    assert trajectory[79].stop_required and trajectory[79].target_speed_mps == 0.0
    assert all(point.target_speed_mps <= 0.0 for point in trajectory[80:])
    assert trajectory[20].target_speed_mps > trajectory[55].target_speed_mps

    for point in trajectory:
        assert math.isclose(
            point.target_angular_z_radps,
            point.target_speed_mps * point.curvature_1pm,
            abs_tol=1e-12,
        )

    for first, second in zip(trajectory, trajectory[1:]):
        distance_m = math.hypot(second.x_cm - first.x_cm, second.y_cm - first.y_cm) / 100.0
        first_speed = abs(first.target_speed_mps)
        second_speed = abs(second.target_speed_mps)
        assert second_speed**2 <= first_speed**2 + 2.0 * 0.05 * distance_m + 1e-12
        assert first_speed**2 <= second_speed**2 + 2.0 * 0.05 * distance_m + 1e-12

    print("Trajectory profile regression passed")
    print(f"Trajectory points: {len(trajectory)}")
    print(f"Straight speed: {trajectory[20].target_speed_mps:.3f} m/s")
    print(f"Curve speed: {trajectory[55].target_speed_mps:.3f} m/s")
    print(f"Direction-change stop index: 79")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
