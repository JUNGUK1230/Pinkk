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
        self.endpoints = self.config["endpoints"]
        self.allowed_transitions = self.config["allowed_transitions"]
        self.endpoint_radius_cm = endpoint_radius_cm

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
        selected = list(points)
        direction = points[0].direction
        selected.insert(
            0,
            FixedRoutePoint(
                x_cm=current_pose[0],
                y_cm=current_pose[1],
                yaw_rad=current_pose[2],
                direction=direction,
            ),
        )
        return FixedRouteSelection(
            source=detected_location,
            target=target,
            detected_location=detected_location,
            join_index=0,
            join_distance_cm=join_distance,
            points=tuple(selected),
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
