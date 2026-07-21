"""Convert a dense Hybrid A* path into a controller-ready speed trajectory."""

from dataclasses import dataclass
import math
from typing import Sequence

try:
    # Package import: from src import build_trajectory_profile
    from .hybrid_astar_planner import HybridState
except ImportError:
    # Script import: PROJECT_ROOT/src is appended to sys.path by test scripts.
    from hybrid_astar_planner import HybridState


@dataclass(frozen=True)
class TrajectoryPoint:
    """One path pose with curvature and signed velocity commands."""

    x_cm: float
    y_cm: float
    yaw_rad: float
    direction: int
    steer_rad: float
    curvature_1pm: float
    target_speed_mps: float
    target_angular_z_radps: float
    stop_required: bool


def build_trajectory_profile(
    path: Sequence[HybridState],
    wheelbase_cm: float,
    max_steer_rad: float | None = None,
    max_forward_speed_mps: float = 0.05,
    max_reverse_speed_mps: float = 0.03,
    min_curve_forward_speed_mps: float = 0.02,
    min_curve_reverse_speed_mps: float = 0.015,
    max_angular_speed_radps: float = 0.5,
    max_acceleration_mps2: float = 0.05,
    max_deceleration_mps2: float = 0.05,
) -> list[TrajectoryPoint]:
    """Create curvature-aware speeds with mandatory stops at direction changes.

    Speeds are signed: positive means forward and negative means reverse. The
    forward pass limits acceleration from a stop, while the backward pass limits
    deceleration into the goal or a forward/reverse cusp.
    """
    _validate_parameters(
        wheelbase_cm,
        max_forward_speed_mps,
        max_reverse_speed_mps,
        min_curve_forward_speed_mps,
        min_curve_reverse_speed_mps,
        max_angular_speed_radps,
        max_acceleration_mps2,
        max_deceleration_mps2,
    )
    if not path:
        return []
    if any(state.direction not in (-1, 1) for state in path):
        raise ValueError("every Hybrid path direction must be either -1 or 1")

    wheelbase_m = wheelbase_cm / 100.0
    curvatures = [math.tan(state.steer_rad) / wheelbase_m for state in path]
    if max_steer_rad is not None and max_steer_rad < 0:
        raise ValueError("max_steer_rad must be zero or positive")
    reference_maximum_curvature = (
        abs(math.tan(max_steer_rad) / wheelbase_m)
        if max_steer_rad is not None
        else max((abs(value) for value in curvatures), default=0.0)
    )

    # The stop is attached to the final point of the old gear segment. The next
    # point already has the new direction and accelerates away from zero.
    stop_indices = {0, len(path) - 1}
    stop_indices.update(
        index
        for index in range(len(path) - 1)
        if path[index].direction != path[index + 1].direction
    )

    speed_limits: list[float] = []
    for index, (state, curvature) in enumerate(zip(path, curvatures)):
        if index in stop_indices:
            speed_limits.append(0.0)
            continue
        if state.direction > 0:
            straight_speed = max_forward_speed_mps
            tight_curve_speed = min_curve_forward_speed_mps
        else:
            straight_speed = max_reverse_speed_mps
            tight_curve_speed = min_curve_reverse_speed_mps

        curvature_ratio = (
            min(abs(curvature) / reference_maximum_curvature, 1.0)
            if reference_maximum_curvature > 1e-12
            else 0.0
        )
        speed = straight_speed - (
            straight_speed - tight_curve_speed
        ) * curvature_ratio
        if abs(curvature) > 1e-12:
            speed = min(speed, max_angular_speed_radps / abs(curvature))
        speed_limits.append(max(0.0, speed))

    distances_m = [
        math.hypot(second.x_cm - first.x_cm, second.y_cm - first.y_cm) / 100.0
        for first, second in zip(path, path[1:])
    ]
    speed_magnitudes = speed_limits.copy()

    # v_next^2 <= v_current^2 + 2*a*distance
    for index in range(1, len(speed_magnitudes)):
        reachable_speed = math.sqrt(
            max(
                0.0,
                speed_magnitudes[index - 1] ** 2
                + 2.0 * max_acceleration_mps2 * distances_m[index - 1],
            )
        )
        speed_magnitudes[index] = min(speed_magnitudes[index], reachable_speed)

    # v_current^2 <= v_next^2 + 2*d*distance
    for index in range(len(speed_magnitudes) - 2, -1, -1):
        reachable_speed = math.sqrt(
            max(
                0.0,
                speed_magnitudes[index + 1] ** 2
                + 2.0 * max_deceleration_mps2 * distances_m[index],
            )
        )
        speed_magnitudes[index] = min(speed_magnitudes[index], reachable_speed)

    trajectory: list[TrajectoryPoint] = []
    for index, (state, curvature, speed_magnitude) in enumerate(
        zip(path, curvatures, speed_magnitudes)
    ):
        signed_speed = state.direction * speed_magnitude
        trajectory.append(
            TrajectoryPoint(
                x_cm=state.x_cm,
                y_cm=state.y_cm,
                yaw_rad=state.yaw_rad,
                direction=state.direction,
                steer_rad=state.steer_rad,
                curvature_1pm=curvature,
                target_speed_mps=signed_speed,
                target_angular_z_radps=signed_speed * curvature,
                stop_required=index in stop_indices,
            )
        )
    return trajectory


def _validate_parameters(
    wheelbase_cm: float,
    max_forward_speed_mps: float,
    max_reverse_speed_mps: float,
    min_curve_forward_speed_mps: float,
    min_curve_reverse_speed_mps: float,
    max_angular_speed_radps: float,
    max_acceleration_mps2: float,
    max_deceleration_mps2: float,
) -> None:
    """Reject unsafe or internally inconsistent trajectory settings."""
    positive_values = {
        "wheelbase_cm": wheelbase_cm,
        "max_forward_speed_mps": max_forward_speed_mps,
        "max_reverse_speed_mps": max_reverse_speed_mps,
        "max_angular_speed_radps": max_angular_speed_radps,
        "max_acceleration_mps2": max_acceleration_mps2,
        "max_deceleration_mps2": max_deceleration_mps2,
    }
    for name, value in positive_values.items():
        if value <= 0:
            raise ValueError(f"{name} must be positive")
    if not 0 <= min_curve_forward_speed_mps <= max_forward_speed_mps:
        raise ValueError(
            "min_curve_forward_speed_mps must be between zero and forward maximum"
        )
    if not 0 <= min_curve_reverse_speed_mps <= max_reverse_speed_mps:
        raise ValueError(
            "min_curve_reverse_speed_mps must be between zero and reverse maximum"
        )
