"""Check fixed yaw, fixed-route selection, and the four-field ROS route contract."""

from pathlib import Path
import sys
import time

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
CENTRAL_ROOT = PROJECT_ROOT.parent
REPO_ROOT = CENTRAL_ROOT.parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from fixed_route_selector import FixedRouteSelector  # noqa: E402
from src.central_control.overhead_vision.localization.live_localization import (  # noqa: E402
    IntegratedPlanningController,
    RoutePublishScheduler,
)
from src.central_control.overhead_vision.localization.scene_localizer import (  # noqa: E402
    AffineBevToLidar,
    Detection,
    EgoVehicleTracker,
)
from src.central_control.overhead_vision.path_planning.direct_ros_publisher import (  # noqa: E402
    TRAJECTORY_FIELDS,
)


def main() -> int:
    fixed_yaw = 0.8063
    transform = AffineBevToLidar(
        CENTRAL_ROOT / "camera_tools/first_map/camera_to_lidar_rigid_registration.npz"
    )
    tracker = EgoVehicleTracker(
        transform,
        rear_axle_offset_cm=4.0,
        fixed_heading_resolver=lambda _center: fixed_yaw,
    )
    polygon = np.asarray(
        [[80.0, 105.0], [120.0, 105.0], [120.0, 135.0], [80.0, 135.0]]
    )
    vehicle = tracker.update(
        [Detection("car", 0.95, polygon, (80.0, 105.0, 120.0, 135.0), 1)]
    )
    assert vehicle is not None
    assert vehicle.planning_ready
    assert abs(vehicle.yaw_rad - fixed_yaw) < 1e-9

    selector = FixedRouteSelector(
        PROJECT_ROOT / "config/fixed_mission_routes.yaml",
        PROJECT_ROOT / "output",
    )
    controller = IntegratedPlanningController(selector)
    start = (68.1221, 23.7980, fixed_yaw)
    outcome = controller._plan_request(
        frame_index=1,
        observed_at_unix_sec=time.time(),
        slot_name="C2",
        start_pose_cm=start,
        goal_pose_cm=(0.0, 0.0, 0.0),
        alternative_goal_pose_cm=(0.0, 0.0, 0.0),
    )
    controller.close()
    assert outcome.trajectory
    assert outcome.adjusted_start == start
    assert TRAJECTORY_FIELDS == ("x_m", "y_m", "yaw_rad", "direction")
    assert not hasattr(outcome.trajectory[0], "target_speed_mps")
    assert not hasattr(outcome.trajectory[0], "steer_rad")

    scheduler = RoutePublishScheduler(1.0)
    assert not scheduler.due(10.0, has_route=False)
    assert scheduler.due(10.1, has_route=True)
    assert not scheduler.due(11.0, has_route=True)
    assert scheduler.due(11.1, has_route=True)
    assert scheduler.due(11.2, has_route=True, is_new_route=True)
    assert not scheduler.due(11.3, has_route=False)
    assert scheduler.due(11.4, has_route=True)

    print("Fixed yaw planning-ready check passed")
    print(f"Fixed route bridge points: {len(outcome.trajectory)}")
    print(f"ROS trajectory fields: {', '.join(TRAJECTORY_FIELDS)}")
    print("Route republish scheduler: immediate + 1.0 sec periodic")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
