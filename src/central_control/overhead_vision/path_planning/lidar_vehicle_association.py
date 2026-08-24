"""Namespace별 LiDAR pose와 카메라 track의 영속 차량 ID를 연결한다."""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import math
import time
from typing import Mapping, Sequence

import numpy as np

try:
    from ....vehicle_control.heading_fusion import LidarMapHeadingMatcher, PoseMatch
except ImportError:  # direct script compatibility
    from vehicle_control.heading_fusion import LidarMapHeadingMatcher, PoseMatch


@dataclass(frozen=True)
class VehicleTrackAssociation:
    vehicle_to_track: Mapping[str, int]
    lidar_poses: Mapping[str, PoseMatch]
    total_score_m: float
    assignment_margin_m: float
    confirmed: bool


class LidarVehicleAssociator:
    """LiDAR-map pose와 camera 후보를 일대일 최소 비용으로 연결한다."""

    def __init__(
        self,
        matcher: LidarMapHeadingMatcher,
        maximum_match_score_m: float = 0.08,
        minimum_assignment_margin_m: float = 0.01,
        required_confirmations: int = 2,
        confirmed_mapping_ttl_sec: float = 2.0,
        position_search_half_width_m: float = 0.25,
    ) -> None:
        if maximum_match_score_m <= 0.0:
            raise ValueError("maximum match score must be positive")
        if minimum_assignment_margin_m < 0.0:
            raise ValueError("minimum assignment margin must be nonnegative")
        if required_confirmations < 1:
            raise ValueError("required confirmations must be positive")
        if confirmed_mapping_ttl_sec <= 0.0:
            raise ValueError("confirmed mapping TTL must be positive")
        if position_search_half_width_m <= 0.0:
            raise ValueError("position search half width must be positive")
        self.matcher = matcher
        self.maximum_match_score_m = float(maximum_match_score_m)
        self.minimum_assignment_margin_m = float(minimum_assignment_margin_m)
        self.required_confirmations = int(required_confirmations)
        self.confirmed_mapping_ttl_sec = float(confirmed_mapping_ttl_sec)
        self.position_search_half_width_m = float(position_search_half_width_m)
        self._pending_mapping: tuple[tuple[str, int], ...] | None = None
        self._pending_count = 0
        self._confirmed: VehicleTrackAssociation | None = None
        self._confirmed_at = 0.0

    def associate(
        self,
        scans_by_vehicle: Mapping[str, np.ndarray],
        tracked_vehicles: Sequence[object],
        now: float | None = None,
    ) -> VehicleTrackAssociation | None:
        """최신 scan과 보이는 camera track으로 신뢰 가능한 ID 매핑을 만든다."""
        timestamp = time.monotonic() if now is None else float(now)
        visible = tuple(
            sorted(
                (
                    vehicle
                    for vehicle in tracked_vehicles
                    if bool(getattr(vehicle, "visible", True))
                ),
                key=lambda vehicle: int(getattr(vehicle, "track_id")),
            )
        )
        vehicle_ids = tuple(sorted(scans_by_vehicle))
        if not vehicle_ids or len(visible) < len(vehicle_ids):
            return self._valid_cached(timestamp, visible)

        matches: dict[tuple[str, int], PoseMatch] = {}
        for vehicle_id in vehicle_ids:
            points = scans_by_vehicle[vehicle_id]
            for tracked in visible:
                track_id = int(getattr(tracked, "track_id"))
                position_cm = getattr(tracked, "position_cm")
                match = self.matcher.match_pose_near(
                    float(position_cm[0]) / 100.0,
                    float(position_cm[1]) / 100.0,
                    points,
                    position_half_width_m=self.position_search_half_width_m,
                )
                if match is not None and match.score_m <= self.maximum_match_score_m:
                    matches[(vehicle_id, track_id)] = match

        ranked_assignments: list[
            tuple[float, tuple[tuple[str, int], ...], dict[str, PoseMatch]]
        ] = []
        for selected_tracks in itertools.permutations(visible, len(vehicle_ids)):
            pairs = tuple(
                (vehicle_id, int(getattr(track, "track_id")))
                for vehicle_id, track in zip(vehicle_ids, selected_tracks)
            )
            if any(pair not in matches for pair in pairs):
                continue
            poses = {vehicle_id: matches[(vehicle_id, track_id)] for vehicle_id, track_id in pairs}
            total_score = sum(pose.score_m for pose in poses.values())
            ranked_assignments.append((total_score, pairs, poses))
        ranked_assignments.sort(key=lambda item: item[0])
        if not ranked_assignments:
            return self._valid_cached(timestamp, visible)

        best_score, best_pairs, best_poses = ranked_assignments[0]
        assignment_margin = (
            ranked_assignments[1][0] - best_score
            if len(ranked_assignments) > 1
            else math.inf
        )
        if assignment_margin < self.minimum_assignment_margin_m:
            return self._valid_cached(timestamp, visible)

        if best_pairs == self._pending_mapping:
            self._pending_count += 1
        else:
            self._pending_mapping = best_pairs
            self._pending_count = 1
        candidate = VehicleTrackAssociation(
            vehicle_to_track=dict(best_pairs),
            lidar_poses=best_poses,
            total_score_m=float(best_score),
            assignment_margin_m=float(assignment_margin),
            confirmed=self._pending_count >= self.required_confirmations,
        )
        if candidate.confirmed:
            self._confirmed = candidate
            self._confirmed_at = timestamp
            return candidate
        return self._valid_cached(timestamp, visible)

    def _valid_cached(
        self,
        now: float,
        visible: Sequence[object],
    ) -> VehicleTrackAssociation | None:
        if self._confirmed is None:
            return None
        if now - self._confirmed_at > self.confirmed_mapping_ttl_sec:
            self._confirmed = None
            return None
        visible_ids = {int(getattr(vehicle, "track_id")) for vehicle in visible}
        if not set(self._confirmed.vehicle_to_track.values()).issubset(visible_ids):
            return None
        return self._confirmed
