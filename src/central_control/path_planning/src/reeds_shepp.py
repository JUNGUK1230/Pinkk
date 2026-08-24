"""Dependency-free Reeds-Shepp shortest-path generation for parking poses.

The closed-form path families and symmetry expansion are adapted from the
MIT-licensed PythonRobotics Reeds-Shepp sample by Atsushi Sakai and
contributors. See ``../THIRD_PARTY_NOTICES.md`` for attribution.

This module is deliberately independent from Hybrid A* and occupancy grids.
Collision checking and analytic expansion integration are later stages.
"""

from dataclasses import dataclass
import math
from typing import Callable, Iterator


Pose = tuple[float, float, float]
PathFormula = Callable[
    [float, float, float],
    tuple[bool, list[float], list[str]],
]


@dataclass(frozen=True)
class ReedsSheppSegment:
    """One constant-curvature segment with signed physical length."""

    mode: str
    length_cm: float

    @property
    def direction(self) -> int:
        return 1 if self.length_cm >= 0.0 else -1


@dataclass(frozen=True)
class ReedsSheppPose:
    """One sampled pose and the segment command used to reach it."""

    x_cm: float
    y_cm: float
    yaw_rad: float
    direction: int
    segment_mode: str


@dataclass(frozen=True)
class ReedsSheppPath:
    """Shortest valid candidate between two poses."""

    segments: tuple[ReedsSheppSegment, ...]
    poses: tuple[ReedsSheppPose, ...]
    total_length_cm: float


@dataclass(frozen=True)
class _Candidate:
    """Normalized path candidate; arc lengths use a unit turning radius."""

    lengths: tuple[float, ...]
    modes: tuple[str, ...]

    @property
    def normalized_length(self) -> float:
        return sum(abs(length) for length in self.lengths)


class ReedsSheppPlanner:
    """Generate shortest forward/reverse paths with bounded curvature."""

    def __init__(self, turning_radius_cm: float, step_size_cm: float = 0.5) -> None:
        if turning_radius_cm <= 0.0:
            raise ValueError("turning_radius_cm must be positive")
        if step_size_cm <= 0.0:
            raise ValueError("step_size_cm must be positive")
        self.turning_radius_cm = turning_radius_cm
        self.step_size_cm = step_size_cm
        self.max_curvature_1pcm = 1.0 / turning_radius_cm

    def plan(self, start: Pose, goal: Pose) -> ReedsSheppPath | None:
        """Return the shortest sampled path, or None if no formula is valid."""
        paths = self.plan_candidates(start, goal)
        return paths[0] if paths else None

    def plan_candidates(self, start: Pose, goal: Pose) -> list[ReedsSheppPath]:
        """Return all valid sampled candidates in ascending length order.

        Hybrid A* analytic expansion needs more than the mathematical shortest
        path: if that path collides, a slightly longer symmetric candidate may
        still provide a safe connection to the exact goal pose.
        """
        return list(self.iter_candidates(start, goal))

    def iter_candidates(self, start: Pose, goal: Pose) -> Iterator[ReedsSheppPath]:
        """Yield sampled candidates shortest-first instead of building all at once.

        Hybrid A* usually accepts or rejects only the first few candidates. Lazy
        sampling avoids constructing every longer 0.5 cm path at each expansion.
        """
        _validate_pose("start", start)
        _validate_pose("goal", goal)
        if (
            math.hypot(goal[0] - start[0], goal[1] - start[1]) <= 1e-12
            and abs(_angle_difference(goal[2], start[2])) <= 1e-12
        ):
            pose = ReedsSheppPose(
                start[0], start[1], _normalize_yaw(start[2]), 1, "S"
            )
            yield ReedsSheppPath((), (pose,), 0.0)
            return

        candidates = self._generate_candidates(start, goal)
        for candidate in sorted(
            candidates,
            key=lambda value: value.normalized_length,
        ):
            path = self._sample_candidate(start, goal, candidate)
            if path is not None:
                yield path

    def _generate_candidates(self, start: Pose, goal: Pose) -> list[_Candidate]:
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        cos_yaw = math.cos(start[2])
        sin_yaw = math.sin(start[2])
        x = (cos_yaw * dx + sin_yaw * dy) * self.max_curvature_1pcm
        y = (-sin_yaw * dx + cos_yaw * dy) * self.max_curvature_1pcm
        phi = _angle_difference(goal[2], start[2])
        candidates: list[_Candidate] = []

        path_formulas: tuple[PathFormula, ...] = (
            _left_straight_left,
            _left_straight_right,
            _left_x_right_x_left,
            _left_x_right_left,
            _left_right_x_left,
            _left_right_x_left_right,
            _left_x_right_left_x_right,
            _left_x_right90_straight_left,
            _left_x_right90_straight_right,
            _left_straight_right90_x_left,
            _left_straight_left90_x_right,
            _left_x_right90_straight_left90_x_right,
        )

        # Time reversal and reflection expand the base formula set into the
        # complete symmetric Reeds-Shepp candidate family.
        for formula in path_formulas:
            valid, lengths, modes = formula(x, y, phi)
            if valid:
                _add_candidate(candidates, lengths, modes)

            valid, lengths, modes = formula(-x, y, -phi)
            if valid:
                _add_candidate(candidates, _time_flip(lengths), modes)

            valid, lengths, modes = formula(x, -y, -phi)
            if valid:
                _add_candidate(candidates, lengths, _reflect(modes))

            valid, lengths, modes = formula(-x, -y, phi)
            if valid:
                _add_candidate(
                    candidates,
                    _time_flip(lengths),
                    _reflect(modes),
                )
        return candidates

    def _sample_candidate(
        self,
        start: Pose,
        goal: Pose,
        candidate: _Candidate,
    ) -> ReedsSheppPath | None:
        local_x = 0.0
        local_y = 0.0
        local_yaw = 0.0
        local_samples: list[tuple[float, float, float, int, str]] = []
        normalized_step = self.step_size_cm * self.max_curvature_1pcm

        for segment_index, (length, mode) in enumerate(
            zip(candidate.lengths, candidate.modes)
        ):
            distances = _sample_distances(length, normalized_step)
            segment_start = (local_x, local_y, local_yaw)
            for distance_index, distance in enumerate(distances):
                x, y, yaw = _interpolate(
                    distance,
                    mode,
                    self.max_curvature_1pcm,
                    segment_start,
                )
                # Keep a single sample at a segment boundary. It retains the old
                # gear, and the first moving point of the next segment gets the new gear.
                if segment_index > 0 and distance_index == 0:
                    continue
                local_samples.append(
                    (x, y, yaw, 1 if length >= 0.0 else -1, mode)
                )
            local_x, local_y, local_yaw = _interpolate(
                length,
                mode,
                self.max_curvature_1pcm,
                segment_start,
            )

        cos_yaw = math.cos(start[2])
        sin_yaw = math.sin(start[2])
        poses = [
            ReedsSheppPose(
                x_cm=start[0] + cos_yaw * x - sin_yaw * y,
                y_cm=start[1] + sin_yaw * x + cos_yaw * y,
                yaw_rad=_normalize_yaw(yaw + start[2]),
                direction=direction,
                segment_mode=mode,
            )
            for x, y, yaw, direction, mode in local_samples
        ]
        if not poses:
            return None

        endpoint = poses[-1]
        position_error = math.hypot(endpoint.x_cm - goal[0], endpoint.y_cm - goal[1])
        yaw_error = abs(_angle_difference(endpoint.yaw_rad, goal[2]))
        if position_error > 1e-6 or yaw_error > 1e-6:
            return None

        # Snap only after validating the formula to remove harmless floating error.
        poses[-1] = ReedsSheppPose(
            goal[0],
            goal[1],
            _normalize_yaw(goal[2]),
            endpoint.direction,
            endpoint.segment_mode,
        )
        segments = tuple(
            ReedsSheppSegment(mode, length / self.max_curvature_1pcm)
            for length, mode in zip(candidate.lengths, candidate.modes)
        )
        return ReedsSheppPath(
            segments=segments,
            poses=tuple(poses),
            total_length_cm=sum(abs(segment.length_cm) for segment in segments),
        )


def _left_straight_left(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if 0.0 <= t <= math.pi:
        v = _mod2pi(phi - t)
        if 0.0 <= v <= math.pi:
            return True, [t, u, v], ["L", "S", "L"]
    return False, [], []


def _left_straight_right(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    squared = u1**2
    if squared >= 4.0:
        u = math.sqrt(squared - 4.0)
        theta = math.atan2(2.0, u)
        t = _mod2pi(t1 + theta)
        v = _mod2pi(t - phi)
        if t >= 0.0 and v >= 0.0:
            return True, [t, u, v], ["L", "S", "R"]
    return False, [], []


def _left_x_right_x_left(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    radius, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if radius <= 4.0:
        angle = math.acos(_clamp(0.25 * radius, -1.0, 1.0))
        t = _mod2pi(angle + theta + math.pi / 2.0)
        u = _mod2pi(math.pi - 2.0 * angle)
        v = _mod2pi(phi - t - u)
        return True, [t, -u, v], ["L", "R", "L"]
    return False, [], []


def _left_x_right_left(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    radius, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if radius <= 4.0:
        angle = math.acos(_clamp(0.25 * radius, -1.0, 1.0))
        t = _mod2pi(angle + theta + math.pi / 2.0)
        u = _mod2pi(math.pi - 2.0 * angle)
        v = _mod2pi(-phi + t + u)
        return True, [t, -u, -v], ["L", "R", "L"]
    return False, [], []


def _left_right_x_left(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    radius, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if 1e-12 < radius <= 4.0:
        u = math.acos(_clamp(1.0 - radius**2 * 0.125, -1.0, 1.0))
        angle = math.asin(_clamp(2.0 * math.sin(u) / radius, -1.0, 1.0))
        t = _mod2pi(-angle + theta + math.pi / 2.0)
        v = _mod2pi(t - u - phi)
        return True, [t, u, -v], ["L", "R", "L"]
    return False, [], []


def _left_right_x_left_right(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    radius, theta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if radius <= 2.0:
        angle = math.acos(_clamp((radius + 2.0) * 0.25, -1.0, 1.0))
        t = _mod2pi(theta + angle + math.pi / 2.0)
        u = _mod2pi(angle)
        v = _mod2pi(phi - t + 2.0 * u)
        if t >= 0.0 and u >= 0.0 and v >= 0.0:
            return True, [t, u, -u, -v], ["L", "R", "L", "R"]
    return False, [], []


def _left_x_right_left_x_right(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    radius, theta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    value = (20.0 - radius**2) / 16.0
    if 1e-12 < radius and 0.0 <= value <= 1.0:
        u = math.acos(value)
        angle = math.asin(_clamp(2.0 * math.sin(u) / radius, -1.0, 1.0))
        t = _mod2pi(theta + angle + math.pi / 2.0)
        v = _mod2pi(t - phi)
        if t >= 0.0 and v >= 0.0:
            return True, [t, -u, -u, v], ["L", "R", "L", "R"]
    return False, [], []


def _left_x_right90_straight_left(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    radius, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if radius >= 2.0:
        root = math.sqrt(max(0.0, radius**2 - 4.0))
        u = root - 2.0
        angle = math.atan2(2.0, root)
        t = _mod2pi(theta + angle + math.pi / 2.0)
        v = _mod2pi(t - phi + math.pi / 2.0)
        if t >= 0.0 and v >= 0.0:
            return True, [t, -math.pi / 2.0, -u, -v], ["L", "R", "S", "L"]
    return False, [], []


def _left_straight_right90_x_left(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    radius, theta = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if radius >= 2.0:
        root = math.sqrt(max(0.0, radius**2 - 4.0))
        u = root - 2.0
        angle = math.atan2(root, 2.0)
        t = _mod2pi(theta - angle + math.pi / 2.0)
        v = _mod2pi(t - phi - math.pi / 2.0)
        if t >= 0.0 and v >= 0.0:
            return True, [t, u, math.pi / 2.0, -v], ["L", "S", "R", "L"]
    return False, [], []


def _left_x_right90_straight_right(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    radius, theta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if radius >= 2.0:
        t = _mod2pi(theta + math.pi / 2.0)
        u = radius - 2.0
        v = _mod2pi(phi - t - math.pi / 2.0)
        if t >= 0.0 and v >= 0.0:
            return True, [t, -math.pi / 2.0, -u, -v], ["L", "R", "S", "R"]
    return False, [], []


def _left_straight_left90_x_right(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    radius, theta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if radius >= 2.0:
        t = _mod2pi(theta)
        u = radius - 2.0
        v = _mod2pi(phi - t - math.pi / 2.0)
        if t >= 0.0 and v >= 0.0:
            return True, [t, u, math.pi / 2.0, -v], ["L", "S", "L", "R"]
    return False, [], []


def _left_x_right90_straight_left90_x_right(
    x: float, y: float, phi: float
) -> tuple[bool, list[float], list[str]]:
    radius, theta = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    if radius >= 4.0:
        root = math.sqrt(max(0.0, radius**2 - 4.0))
        u = root - 4.0
        angle = math.atan2(2.0, root)
        t = _mod2pi(theta + angle + math.pi / 2.0)
        v = _mod2pi(t - phi)
        if t >= 0.0 and v >= 0.0:
            return (
                True,
                [t, -math.pi / 2.0, -u, -math.pi / 2.0, v],
                ["L", "R", "S", "L", "R"],
            )
    return False, [], []


def _add_candidate(
    candidates: list[_Candidate],
    lengths: list[float],
    modes: list[str],
) -> None:
    filtered = [
        (length, mode)
        for length, mode in zip(lengths, modes)
        if abs(length) > 1e-12
    ]
    if not filtered:
        return
    candidate = _Candidate(
        tuple(length for length, _ in filtered),
        tuple(mode for _, mode in filtered),
    )
    if candidate.normalized_length <= 1e-12:
        return
    for existing in candidates:
        if (
            existing.modes == candidate.modes
            and len(existing.lengths) == len(candidate.lengths)
            and all(
                abs(first - second) <= 1e-10
                for first, second in zip(existing.lengths, candidate.lengths)
            )
        ):
            return
    candidates.append(candidate)


def _sample_distances(length: float, step: float) -> list[float]:
    direction = 1.0 if length >= 0.0 else -1.0
    absolute_length = abs(length)
    sample_count = int(math.floor(absolute_length / step))
    distances = [direction * index * step for index in range(sample_count + 1)]
    if not math.isclose(distances[-1], length, abs_tol=1e-12):
        distances.append(length)
    else:
        distances[-1] = length
    return distances


def _interpolate(
    distance: float,
    mode: str,
    max_curvature: float,
    origin: Pose,
) -> Pose:
    origin_x, origin_y, origin_yaw = origin
    if mode == "S":
        return (
            origin_x + distance / max_curvature * math.cos(origin_yaw),
            origin_y + distance / max_curvature * math.sin(origin_yaw),
            origin_yaw,
        )

    local_dx = math.sin(distance) / max_curvature
    if mode == "L":
        local_dy = (1.0 - math.cos(distance)) / max_curvature
        yaw = origin_yaw + distance
    elif mode == "R":
        local_dy = -(1.0 - math.cos(distance)) / max_curvature
        yaw = origin_yaw - distance
    else:
        raise ValueError(f"unsupported Reeds-Shepp segment mode: {mode}")
    return (
        origin_x + math.cos(origin_yaw) * local_dx - math.sin(origin_yaw) * local_dy,
        origin_y + math.sin(origin_yaw) * local_dx + math.cos(origin_yaw) * local_dy,
        yaw,
    )


def _time_flip(lengths: list[float]) -> list[float]:
    return [-length for length in lengths]


def _reflect(modes: list[str]) -> list[str]:
    return [{"L": "R", "R": "L", "S": "S"}[mode] for mode in modes]


def _polar(x: float, y: float) -> tuple[float, float]:
    return math.hypot(x, y), math.atan2(y, x)


def _mod2pi(angle: float) -> float:
    value = math.fmod(angle, math.copysign(2.0 * math.pi, angle))
    if value < -math.pi:
        value += 2.0 * math.pi
    elif value > math.pi:
        value -= 2.0 * math.pi
    return value


def _normalize_yaw(yaw_rad: float) -> float:
    return (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi


def _angle_difference(first: float, second: float) -> float:
    return _normalize_yaw(first - second)


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _validate_pose(label: str, pose: Pose) -> None:
    if len(pose) != 3 or not all(math.isfinite(value) for value in pose):
        raise ValueError(f"{label} must contain three finite values")
