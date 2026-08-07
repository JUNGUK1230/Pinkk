"""LiDAR map 절대 heading과 IMU 상대 yaw 융합을 검사한다."""

import math
from pathlib import Path
import sys
import tempfile

import cv2
import numpy as np


SRC_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC_ROOT))

from vehicle_control.heading_fusion import (  # noqa: E402
    ImuLidarHeadingFusion,
    LidarMapHeadingMatcher,
    angle_difference,
    motion_heading,
)


def main() -> int:
    with tempfile.TemporaryDirectory() as directory:
        map_path = Path(directory) / "asymmetric_map.png"
        image = np.full((200, 220), 255, dtype=np.uint8)
        cv2.line(image, (15, 20), (205, 20), 0, 2)
        cv2.line(image, (205, 20), (205, 180), 0, 2)
        cv2.line(image, (40, 80), (150, 80), 0, 2)
        cv2.line(image, (40, 80), (40, 155), 0, 2)
        cv2.imwrite(str(map_path), image)

        matcher = LidarMapHeadingMatcher(
            map_path,
            resolution_m_per_px=0.01,
            lidar_x_m=-0.017,
            scan_frame_yaw_deg=180.0,
            minimum_points=20,
            trimmed_fraction=0.9,
        )
        position = np.asarray((1.05, 1.15), dtype=np.float64)
        expected_yaw = math.radians(37.0)
        cosine, sine = math.cos(expected_yaw), math.sin(expected_yaw)
        sensor = position + np.asarray(
            (cosine * -0.017, sine * -0.017),
            dtype=np.float64,
        )
        wall_pixels = np.asarray(
            [
                *[(x, 20) for x in range(20, 201, 5)],
                *[(205, y) for y in range(25, 181, 5)],
                *[(x, 80) for x in range(45, 151, 5)],
                *[(40, y) for y in range(85, 156, 5)],
            ],
            dtype=np.float64,
        )
        world = wall_pixels * 0.01
        delta = world - sensor
        vehicle_points = np.column_stack(
            (
                cosine * delta[:, 0] + sine * delta[:, 1],
                -sine * delta[:, 0] + cosine * delta[:, 1],
            )
        )
        match = matcher.match_global(position[0], position[1], vehicle_points)
        assert match is not None
        assert abs(angle_difference(match.yaw_rad, expected_yaw)) < math.radians(1.0)
        assert match.score_m < 0.01
        assert match.distinct_margin_m > 0.0005

        scan_points = matcher.scan_points(
            [1.0, 1.0],
            angle_min=0.0,
            angle_increment=math.pi / 2.0,
            range_min=0.05,
            range_max=10.0,
        )
        # minimum_points 때문에 별도 matcher로 변환 방향만 검사한다.
        small_matcher = LidarMapHeadingMatcher(
            map_path,
            resolution_m_per_px=0.01,
            scan_frame_yaw_deg=180.0,
            scan_subsample=1,
            minimum_points=3,
        )
        scan_points = small_matcher.scan_points(
            [1.0, 1.0, 1.0],
            angle_min=0.0,
            angle_increment=math.pi / 2.0,
            range_min=0.05,
            range_max=10.0,
        )
        assert np.allclose(scan_points[0], (-1.0, 0.0), atol=1e-8)
        assert np.allclose(scan_points[1], (0.0, 1.0), atol=1e-8)

    fusion = ImuLidarHeadingFusion(
        lidar_correction_alpha=0.2,
        imu_yaw_sign=-1.0,
    )
    first = fusion.correct(math.radians(20.0), math.radians(37.0))
    assert abs(angle_difference(first, math.radians(37.0))) < 1e-8
    # ROS IMU가 반시계방향으로 +10도 움직이면 image-map yaw는 -10도다.
    predicted = fusion.heading(math.radians(30.0))
    assert predicted is not None
    assert abs(angle_difference(predicted, math.radians(27.0))) < 1e-8
    corrected = fusion.correct(math.radians(30.0), math.radians(28.0))
    assert abs(angle_difference(corrected, math.radians(27.2))) < 1e-8

    assert motion_heading((0.0, 0.0), (0.01, 0.0), 1, 0.015) is None
    forward_motion = motion_heading((0.0, 0.0), (0.02, 0.02), 1, 0.015)
    assert forward_motion is not None
    assert abs(angle_difference(forward_motion, math.radians(45.0))) < 1e-8
    reverse_motion = motion_heading((0.02, 0.02), (0.0, 0.0), -1, 0.015)
    assert reverse_motion is not None
    assert abs(angle_difference(reverse_motion, math.radians(45.0))) < 1e-8

    print("IMU + LiDAR map heading fusion test passed")
    print(f"Recovered global heading: {math.degrees(match.yaw_rad):.2f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
