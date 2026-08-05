"""Check measured camera yaw, fixed-route selection, and ROS route contract."""

from pathlib import Path
from types import SimpleNamespace
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
    route_context,
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
    measured_yaw = transform.axis_yaw_in_lidar((100.0, 120.0), (1.0, 0.0))
    assert abs(vehicle.yaw_rad - measured_yaw) < 1e-9
    assert abs(vehicle.yaw_rad - fixed_yaw) > 1e-3

    selector = FixedRouteSelector(
        PROJECT_ROOT / "config/fixed_mission_routes.yaml",
        PROJECT_ROOT / "output",
    )
    controller = IntegratedPlanningController(selector)
    start = tuple(selector.endpoints["START"]["staging"])
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

    # 다른 차량의 충전 배정 C2를 C1에 있는 ego의 다음 목표로 표시하면
    # 안 된다. C1/C2의 유효한 다음 목표는 P1~P4뿐이다.
    c1_vehicle = SimpleNamespace(
        track_id=7,
        center_cm=(
            float(selector.endpoints["C1"]["goal"][0]) + 3.5,
            float(selector.endpoints["C1"]["goal"][1]),
        ),
        yaw_rad=-2.4521787849,
    )
    c1_tracked = SimpleNamespace(track_id=7, assigned_slot_name="C1")
    unrelated_charge_assignment = SimpleNamespace(
        vehicle_track_id=9,
        target_slot_name="C2",
    )
    c1_scene = SimpleNamespace(
        vehicle=c1_vehicle,
        tracked_vehicles=(c1_tracked,),
        planning_request=None,
        charge_assignment=unrelated_charge_assignment,
    )
    assert route_context(c1_scene, selector) == ("C1", "WAIT")
    c1_scene.planning_request = SimpleNamespace(slot_name="C2")
    assert route_context(c1_scene, selector) == ("C1", "WAIT")
    c1_scene.planning_request = SimpleNamespace(slot_name="P3")
    assert route_context(c1_scene, selector) == ("C1", "P3")

    arrival_controller = IntegratedPlanningController(selector)
    c1_goal = selector.endpoints["C1"]["goal"]
    arrival_controller.outcome = SimpleNamespace(
        trajectory=(
            SimpleNamespace(
                x_cm=float(c1_goal[0]),
                y_cm=float(c1_goal[1]),
            ),
        )
    )
    arrival_controller._active_key = (7, "C1", 0)
    c1_scene.planning_request = None
    # 기존 C1 종점은 새 종점에서 3cm 떨어져 있다. 주차칸에 이미 겹쳤어도
    # 완료 반경 2.5cm 밖이면 경로를 유지해야 한다.
    arrival_controller.update(c1_scene, route_revision=0)
    assert arrival_controller.outcome is not None
    assert arrival_controller._active_key is not None
    assert not arrival_controller.consume_invalidation()
    c1_scene.vehicle = SimpleNamespace(
        track_id=7,
        center_cm=(float(c1_goal[0]), float(c1_goal[1])),
        yaw_rad=float(c1_goal[2]),
    )
    arrival_controller.update(c1_scene, route_revision=0)
    assert arrival_controller.outcome is None
    assert arrival_controller._active_key is None
    assert arrival_controller.consume_invalidation()
    assert not arrival_controller.consume_invalidation()
    arrival_controller.close()

    startup_at_c1_controller = IntegratedPlanningController(selector)
    startup_at_c1_controller.update(c1_scene, route_revision=0)
    assert startup_at_c1_controller.consume_invalidation()
    assert not startup_at_c1_controller.consume_invalidation()
    startup_at_c1_controller.close()

    p8_scene = SimpleNamespace(
        vehicle=SimpleNamespace(
            track_id=8,
            center_cm=tuple(selector.endpoints["P8"]["goal"][:2]),
            yaw_rad=float(selector.endpoints["P8"]["goal"][2]),
        ),
        tracked_vehicles=(
            SimpleNamespace(track_id=8, assigned_slot_name="P8"),
        ),
        planning_request=SimpleNamespace(slot_name="C2"),
        charge_assignment=None,
    )
    assert route_context(p8_scene, selector) == ("P8", "C2")
    p8_scene.planning_request = SimpleNamespace(slot_name="P7")
    assert route_context(p8_scene, selector) == ("P8", "WAIT")

    print("Measured camera yaw + fixed front/back reference check passed")
    print(f"Fixed route bridge points: {len(outcome.trajectory)}")
    print(f"ROS trajectory fields: {', '.join(TRAJECTORY_FIELDS)}")
    print("Route republish scheduler: immediate + 1.0 sec periodic")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
