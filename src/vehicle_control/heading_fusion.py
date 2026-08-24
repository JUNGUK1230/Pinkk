"""상단 카메라 위치에서 LiDAR map 정합과 IMU로 차량 heading을 추정한다."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np


def normalize_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def angle_difference(target: float, source: float) -> float:
    return normalize_angle(target - source)


def motion_heading(
    start: tuple[float, float],
    end: tuple[float, float],
    direction: int,
    minimum_baseline_m: float,
) -> float | None:
    """차량 중심 이동 벡터에서 차체 yaw를 계산한다."""
    if direction not in (-1, 1):
        raise ValueError("motion direction must be -1 or 1")
    if not math.isfinite(minimum_baseline_m) or minimum_baseline_m <= 0.0:
        raise ValueError("motion heading baseline must be positive and finite")
    delta_x = float(end[0]) - float(start[0])
    delta_y = float(end[1]) - float(start[1])
    if not all(math.isfinite(value) for value in (delta_x, delta_y)):
        raise ValueError("motion positions must be finite")
    if math.hypot(delta_x, delta_y) < minimum_baseline_m:
        return None
    yaw = math.atan2(delta_y, delta_x)
    if direction < 0:
        yaw += math.pi
    return normalize_angle(yaw)


def quaternion_yaw_xyzw(x: float, y: float, z: float, w: float) -> float:
    values = (x, y, z, w)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("IMU quaternion contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-8:
        raise ValueError("IMU quaternion norm is zero")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


@dataclass(frozen=True)
class HeadingMatch:
    yaw_rad: float
    score_m: float
    distinct_margin_m: float
    point_count: int


@dataclass(frozen=True)
class PoseMatch:
    position_x_m: float
    position_y_m: float
    yaw_rad: float
    score_m: float
    distinct_margin_m: float
    point_count: int


class LidarMapHeadingMatcher:
    """고정 위치에서 scan endpoint와 occupancy wall의 거리를 최소화한다.

    lidar_map 좌표는 이미지와 동일하게 x가 오른쪽, y가 아래쪽이며 yaw 양수는
    시계방향이다. LaserScan은 ROS 좌표(x 전방, y 좌측)이므로 장착 회전 후 y를
    반전해 이미지 좌표로 변환한다.
    """

    def __init__(
        self,
        map_image_path: str | Path,
        resolution_m_per_px: float,
        occupied_pixel_threshold: int = 100,
        lidar_x_m: float = -0.017,
        lidar_y_m: float = 0.0,
        scan_frame_yaw_deg: float = 180.0,
        minimum_range_m: float = 0.08,
        maximum_range_m: float = 2.5,
        scan_subsample: int = 3,
        trimmed_fraction: float = 0.70,
        outside_penalty_m: float = 0.30,
        minimum_points: int = 25,
    ) -> None:
        image = cv2.imread(str(map_image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f"could not load LiDAR map: {map_image_path}")
        if resolution_m_per_px <= 0.0:
            raise ValueError("map resolution must be positive")
        if scan_subsample < 1 or minimum_points < 3:
            raise ValueError("scan sampling parameters are invalid")
        if not 0.1 <= trimmed_fraction <= 1.0:
            raise ValueError("trimmed_fraction must be in [0.1, 1.0]")
        free_mask = (image > int(occupied_pixel_threshold)).astype(np.uint8)
        self._distance_m = (
            cv2.distanceTransform(free_mask, cv2.DIST_L2, 5)
            * float(resolution_m_per_px)
        )
        self.resolution_m_per_px = float(resolution_m_per_px)
        self.lidar_x_m = float(lidar_x_m)
        self.lidar_y_m = float(lidar_y_m)
        self.scan_frame_yaw_rad = math.radians(float(scan_frame_yaw_deg))
        self.minimum_range_m = float(minimum_range_m)
        self.maximum_range_m = float(maximum_range_m)
        self.scan_subsample = int(scan_subsample)
        self.trimmed_fraction = float(trimmed_fraction)
        self.outside_penalty_m = float(outside_penalty_m)
        self.minimum_points = int(minimum_points)

    def scan_points(
        self,
        ranges: Sequence[float],
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
    ) -> np.ndarray:
        indices = np.arange(0, len(ranges), self.scan_subsample, dtype=np.int32)
        distances = np.asarray(ranges, dtype=np.float64)[indices]
        angles = float(angle_min) + indices * float(angle_increment)
        lower = max(float(range_min), self.minimum_range_m)
        upper = min(float(range_max), self.maximum_range_m)
        valid = np.isfinite(distances) & (distances >= lower) & (distances <= upper)
        distances = distances[valid]
        angles = angles[valid]
        if distances.size < self.minimum_points:
            return np.empty((0, 2), dtype=np.float64)

        scan_x = distances * np.cos(angles)
        scan_y = distances * np.sin(angles)
        cosine = math.cos(self.scan_frame_yaw_rad)
        sine = math.sin(self.scan_frame_yaw_rad)
        # scan frame -> base ROS frame, then ROS left-positive y -> map
        # right/down-positive local y.
        vehicle_x = cosine * scan_x - sine * scan_y
        vehicle_y = -(sine * scan_x + cosine * scan_y)
        return np.column_stack((vehicle_x, vehicle_y))

    def match_global(
        self,
        position_x_m: float,
        position_y_m: float,
        scan_points: np.ndarray,
        coarse_step_deg: float = 3.0,
        refine_half_width_deg: float = 4.0,
        refine_step_deg: float = 0.25,
    ) -> HeadingMatch | None:
        headings = np.arange(-180.0, 180.0, coarse_step_deg, dtype=np.float64)
        coarse = self._rank(
            position_x_m,
            position_y_m,
            scan_points,
            np.radians(headings),
        )
        if not coarse:
            return None
        coarse_best_score, coarse_best_yaw, _ = coarse[0]
        distinct_scores = [
            score
            for score, yaw, _ in coarse[1:]
            if abs(angle_difference(yaw, coarse_best_yaw)) >= math.radians(8.0)
        ]
        refined = self.match_local(
            position_x_m,
            position_y_m,
            scan_points,
            coarse_best_yaw,
            refine_half_width_deg,
            refine_step_deg,
        )
        if refined is None:
            return None
        coarse_margin = (
            min(distinct_scores) - min(coarse_best_score, refined.score_m)
            if distinct_scores
            else 0.0
        )
        return HeadingMatch(
            yaw_rad=refined.yaw_rad,
            score_m=refined.score_m,
            distinct_margin_m=max(refined.distinct_margin_m, coarse_margin),
            point_count=refined.point_count,
        )

    def match_local(
        self,
        position_x_m: float,
        position_y_m: float,
        scan_points: np.ndarray,
        predicted_yaw_rad: float,
        half_width_deg: float = 15.0,
        step_deg: float = 0.5,
    ) -> HeadingMatch | None:
        offsets = np.arange(
            -float(half_width_deg),
            float(half_width_deg) + 0.5 * float(step_deg),
            float(step_deg),
            dtype=np.float64,
        )
        headings = np.asarray(
            [normalize_angle(predicted_yaw_rad + math.radians(value)) for value in offsets]
        )
        ranked = self._rank(
            position_x_m,
            position_y_m,
            scan_points,
            headings,
        )
        if not ranked:
            return None
        best_score, best_yaw, count = ranked[0]
        distinct = [
            score
            for score, yaw, _ in ranked[1:]
            if abs(angle_difference(yaw, best_yaw)) >= math.radians(5.0)
        ]
        margin = (min(distinct) - best_score) if distinct else 0.0
        return HeadingMatch(
            yaw_rad=normalize_angle(best_yaw),
            score_m=float(best_score),
            distinct_margin_m=float(margin),
            point_count=int(count),
        )

    def match_pose_near(
        self,
        initial_x_m: float,
        initial_y_m: float,
        scan_points: np.ndarray,
        position_half_width_m: float = 0.25,
        coarse_position_step_m: float = 0.05,
        coarse_yaw_step_deg: float = 10.0,
        refine_position_half_width_m: float = 0.06,
        refine_position_step_m: float = 0.01,
        refine_yaw_half_width_deg: float = 6.0,
        refine_yaw_step_deg: float = 0.5,
    ) -> PoseMatch | None:
        """카메라 후보 주변에서 LiDAR만으로 x/y/yaw pose를 재정합한다."""
        numeric = (
            initial_x_m,
            initial_y_m,
            position_half_width_m,
            coarse_position_step_m,
            coarse_yaw_step_deg,
            refine_position_half_width_m,
            refine_position_step_m,
            refine_yaw_half_width_deg,
            refine_yaw_step_deg,
        )
        if any(not math.isfinite(float(value)) for value in numeric):
            raise ValueError("pose search parameters must be finite")
        if any(float(value) <= 0.0 for value in numeric[2:]):
            raise ValueError("pose search ranges and steps must be positive")
        if (
            scan_points.ndim != 2
            or scan_points.shape[1:] != (2,)
            or len(scan_points) < self.minimum_points
        ):
            return None

        coarse_offsets = np.arange(
            -position_half_width_m,
            position_half_width_m + 0.5 * coarse_position_step_m,
            coarse_position_step_m,
            dtype=np.float64,
        )
        coarse_headings = np.radians(
            np.arange(-180.0, 180.0, coarse_yaw_step_deg, dtype=np.float64)
        )
        coarse_candidates = self._rank_poses(
            float(initial_x_m),
            float(initial_y_m),
            coarse_offsets,
            coarse_offsets,
            scan_points,
            coarse_headings,
        )
        if not coarse_candidates:
            return None
        _, coarse_x, coarse_y, coarse_yaw, _ = coarse_candidates[0]

        refine_offsets = np.arange(
            -refine_position_half_width_m,
            refine_position_half_width_m + 0.5 * refine_position_step_m,
            refine_position_step_m,
            dtype=np.float64,
        )
        yaw_offsets = np.arange(
            -refine_yaw_half_width_deg,
            refine_yaw_half_width_deg + 0.5 * refine_yaw_step_deg,
            refine_yaw_step_deg,
            dtype=np.float64,
        )
        refine_headings = np.asarray(
            [
                normalize_angle(coarse_yaw + math.radians(float(offset)))
                for offset in yaw_offsets
            ],
            dtype=np.float64,
        )
        refined = self._rank_poses(
            coarse_x,
            coarse_y,
            refine_offsets,
            refine_offsets,
            scan_points,
            refine_headings,
        )
        if not refined:
            return None
        best_score, best_x, best_y, best_yaw, count = refined[0]
        distinct_scores = [
            score
            for score, x_m, y_m, yaw_rad, _ in refined[1:]
            if math.hypot(x_m - best_x, y_m - best_y) >= 0.04
            or abs(angle_difference(yaw_rad, best_yaw)) >= math.radians(5.0)
        ]
        margin = min(distinct_scores) - best_score if distinct_scores else 0.0
        return PoseMatch(
            position_x_m=float(best_x),
            position_y_m=float(best_y),
            yaw_rad=normalize_angle(best_yaw),
            score_m=float(best_score),
            distinct_margin_m=float(margin),
            point_count=int(count),
        )

    def _rank_poses(
        self,
        center_x_m: float,
        center_y_m: float,
        x_offsets_m: np.ndarray,
        y_offsets_m: np.ndarray,
        scan_points: np.ndarray,
        headings: np.ndarray,
    ) -> list[tuple[float, float, float, float, int]]:
        ranked: list[tuple[float, float, float, float, int]] = []
        for y_offset in y_offsets_m:
            for x_offset in x_offsets_m:
                x_m = center_x_m + float(x_offset)
                y_m = center_y_m + float(y_offset)
                for score, yaw_rad, count in self._rank(
                    x_m,
                    y_m,
                    scan_points,
                    headings,
                ):
                    ranked.append((score, x_m, y_m, yaw_rad, count))
        ranked.sort(key=lambda item: item[0])
        return ranked

    def _rank(
        self,
        position_x_m: float,
        position_y_m: float,
        scan_points: np.ndarray,
        headings: np.ndarray,
    ) -> list[tuple[float, float, int]]:
        if (
            scan_points.ndim != 2
            or scan_points.shape[1:] != (2,)
            or len(scan_points) < self.minimum_points
        ):
            return []
        ranked: list[tuple[float, float, int]] = []
        for yaw in headings:
            score, count = self._score(
                float(position_x_m),
                float(position_y_m),
                scan_points,
                float(yaw),
            )
            if math.isfinite(score):
                ranked.append((score, float(yaw), count))
        ranked.sort(key=lambda item: item[0])
        return ranked

    def _score(
        self,
        position_x_m: float,
        position_y_m: float,
        scan_points: np.ndarray,
        yaw_rad: float,
    ) -> tuple[float, int]:
        cosine = math.cos(yaw_rad)
        sine = math.sin(yaw_rad)
        sensor_x = (
            position_x_m + cosine * self.lidar_x_m - sine * self.lidar_y_m
        )
        sensor_y = (
            position_y_m + sine * self.lidar_x_m + cosine * self.lidar_y_m
        )
        world_x = sensor_x + cosine * scan_points[:, 0] - sine * scan_points[:, 1]
        world_y = sensor_y + sine * scan_points[:, 0] + cosine * scan_points[:, 1]
        pixel_x = np.rint(world_x / self.resolution_m_per_px).astype(np.int32)
        pixel_y = np.rint(world_y / self.resolution_m_per_px).astype(np.int32)
        height, width = self._distance_m.shape
        inside = (
            (pixel_x >= 0)
            & (pixel_x < width)
            & (pixel_y >= 0)
            & (pixel_y < height)
        )
        if int(np.count_nonzero(inside)) < self.minimum_points:
            return math.inf, 0
        distances = np.full(len(scan_points), self.outside_penalty_m)
        distances[inside] = self._distance_m[pixel_y[inside], pixel_x[inside]]
        keep = max(
            self.minimum_points,
            int(math.ceil(len(distances) * self.trimmed_fraction)),
        )
        selected = np.partition(distances, keep - 1)[:keep]
        return float(np.mean(selected)), int(np.count_nonzero(inside))


class ImuLidarHeadingFusion:
    """LiDAR 절대 yaw로 IMU 상대 yaw의 map offset을 천천히 보정한다."""

    def __init__(
        self,
        lidar_correction_alpha: float = 0.15,
        imu_yaw_sign: float = -1.0,
    ) -> None:
        if not 0.0 < lidar_correction_alpha <= 1.0:
            raise ValueError("lidar correction alpha must be in (0, 1]")
        if imu_yaw_sign not in (-1.0, 1.0):
            raise ValueError("IMU yaw sign must be -1 or 1")
        self.alpha = float(lidar_correction_alpha)
        self.imu_yaw_sign = float(imu_yaw_sign)
        self._offset_rad: float | None = None

    @property
    def initialized(self) -> bool:
        return self._offset_rad is not None

    def heading(self, imu_yaw_rad: float) -> float | None:
        if self._offset_rad is None:
            return None
        return normalize_angle(
            self.imu_yaw_sign * float(imu_yaw_rad) + self._offset_rad
        )

    def correct(self, imu_yaw_rad: float, lidar_yaw_rad: float) -> float:
        imu_map_yaw = self.imu_yaw_sign * float(imu_yaw_rad)
        if self._offset_rad is None:
            self._offset_rad = normalize_angle(lidar_yaw_rad - imu_map_yaw)
        else:
            predicted = normalize_angle(imu_map_yaw + self._offset_rad)
            self._offset_rad = normalize_angle(
                self._offset_rad
                + self.alpha * angle_difference(lidar_yaw_rad, predicted)
            )
        heading = self.heading(imu_yaw_rad)
        assert heading is not None
        return heading
