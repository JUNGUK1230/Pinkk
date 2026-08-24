"""Select a pre-generated route from a vehicle at START or in a known slot."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path

import yaml


Pose = tuple[float, float, float]


@dataclass(frozen=True)
class FixedRoutePoint:
    x_cm: float
    y_cm: float
    yaw_rad: float
    direction: int


@dataclass(frozen=True)
class FixedRouteSelection:
    source: str
    target: str
    detected_location: str
    join_index: int
    join_distance_cm: float
    points: tuple[FixedRoutePoint, ...]


class FixedRouteSelector:
    """Match a localized START/parking pose to one configured route file."""

    def __init__(
        self,
        config_path: str | Path,
        route_directory: str | Path,
        endpoint_radius_cm: float = 12.0,
    ) -> None:
        if endpoint_radius_cm <= 0.0:
            raise ValueError("endpoint_radius_cm must be positive")
        self.config_path = Path(config_path)
        self.route_directory = Path(route_directory)
        with self.config_path.open(encoding="utf-8") as file:
            self.config = yaml.safe_load(file)
        if self.config.get("trajectory_reference") != "vehicle_center":
            raise ValueError("fixed route trajectory reference must be vehicle_center")
        self.rear_axle_to_center_cm = float(
            self.config["rear_axle_to_center_cm"]
        )
        if self.rear_axle_to_center_cm < 0.0:
            raise ValueError("rear_axle_to_center_cm must not be negative")
        self.rear_axle_endpoints = self.config["endpoints"]
        self.endpoints = {
            name: {
                **endpoint,
                **(
                    {
                        "staging": list(
                            self.to_vehicle_center_pose(endpoint["staging"])
                        )
                    }
                    if "staging" in endpoint
                    else {}
                ),
                **(
                    {
                        "goal": list(
                            self.to_vehicle_center_pose(endpoint["goal"])
                        )
                    }
                    if "goal" in endpoint
                    else {}
                ),
            }
            for name, endpoint in self.rear_axle_endpoints.items()
        }
        self.allowed_transitions = self.config["allowed_transitions"]
        self.endpoint_radius_cm = endpoint_radius_cm

    def to_vehicle_center_pose(self, pose: list[float] | Pose) -> Pose:
        """Rear-axle pose를 동일 yaw의 vehicle-center pose로 변환한다."""
        x_cm, y_cm, yaw_rad = map(float, pose)
        return (
            x_cm + self.rear_axle_to_center_cm * math.cos(yaw_rad),
            y_cm + self.rear_axle_to_center_cm * math.sin(yaw_rad),
            yaw_rad,
        )

    def detect_location(self, pose: Pose) -> str:
        """Return a known endpoint name or ``TRANSIT`` for a road position."""
        candidates: list[tuple[float, str]] = []
        for name, endpoint in self.endpoints.items():
            reference = endpoint.get("goal", endpoint["staging"])
            distance = math.hypot(
                float(reference[0]) - pose[0],
                float(reference[1]) - pose[1],
            )
            candidates.append((distance, name))
        distance, name = min(candidates)
        return name if distance <= self.endpoint_radius_cm else "TRANSIT"

    def select(self, current_pose: Pose, target: str) -> FixedRouteSelection:
        """Detect the source endpoint and return its complete configured route."""
        target = target.upper()
        detected_location = self.detect_location(current_pose)
        if detected_location == "TRANSIT":
            raise ValueError(
                "vehicle is not at START or inside a configured parking endpoint"
            )
        targets = self.allowed_transitions.get(detected_location, ())
        if target not in targets:
            raise ValueError(f"no configured fixed route reaches target '{target}'")

        points = self._load_route(detected_location, target)
        if not points:
            raise RuntimeError(
                f"fixed route data is empty: {detected_location} -> {target}"
            )
        join_distance = math.hypot(
            points[0].x_cm - current_pose[0],
            points[0].y_cm - current_pose[1],
        )
        return FixedRouteSelection(
            source=detected_location,
            target=target,
            detected_location=detected_location,
            join_index=0,
            join_distance_cm=join_distance,
            # 실시간 pose는 차량별 `localization_pose`로 별도 전달된다. 검출 pose를
            # 고정 CSV 앞에 삽입하면 측정 오차 때문에 첫 두 점 사이에 가짜
            # 급회전이 생기므로 원본 경로를 그대로 보낸다.
            points=tuple(points),
        )

    def _load_route(self, source: str, target: str) -> list[FixedRoutePoint]:
        path = self.route_directory / (
            f"fixed_route_{source.lower()}_to_{target.lower()}.csv"
        )
        if not path.exists():
            raise FileNotFoundError(
                f"fixed route is missing; run --generate-all first: {path}"
            )
        with path.open(encoding="utf-8") as file:
            rows = csv.DictReader(file)
            return [
                FixedRoutePoint(
                    x_cm=float(row["x_cm"]),
                    y_cm=float(row["y_cm"]),
                    yaw_rad=float(row["yaw_rad"]),
                    direction=int(row["direction"]),
                )
                for row in rows
            ]
