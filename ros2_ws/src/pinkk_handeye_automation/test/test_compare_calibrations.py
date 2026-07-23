from pathlib import Path
import unittest

import numpy as np

from pinkk_handeye_automation.compare_calibrations import (
    load_calibration,
    matrix_to_quaternion,
    mean_transform,
    quaternion_to_matrix,
    rotation_angle_degrees,
    summarize_transforms,
)


DATA_DIR = (
    Path(__file__).resolve().parents[4]
    / "src/robot_arm/robot_camera/handeye_calibration_1828/data/easy_handeye2"
)


class TestCalibrationMath(unittest.TestCase):
    def test_repository_calibration_difference(self) -> None:
        old, _ = load_calibration(DATA_DIR / "pinkk_eye_in_hand_20260715.calib")
        new, _ = load_calibration(
            DATA_DIR / "pinkk_eye_in_hand_30samples_20260723.calib"
        )
        delta = np.linalg.inv(old) @ new
        self.assertAlmostEqual(
            np.linalg.norm(delta[:3, 3]) * 1000.0, 19.090863590, places=6
        )
        self.assertAlmostEqual(
            rotation_angle_degrees(delta[:3, :3]), 6.173309724, places=6
        )

    def test_quaternion_round_trip(self) -> None:
        expected = np.array((0.1, -0.2, 0.3, 0.9), dtype=float)
        expected /= np.linalg.norm(expected)
        actual = np.array(matrix_to_quaternion(quaternion_to_matrix(expected)))
        self.assertTrue(
            np.allclose(actual, expected) or np.allclose(actual, -expected)
        )

    def test_summary_uses_pose_scatter(self) -> None:
        transforms = [np.eye(4, dtype=float) for _ in range(3)]
        transforms[0][0, 3] = -0.001
        transforms[2][0, 3] = 0.001
        center = mean_transform(transforms)
        summary = summarize_transforms(transforms)
        self.assertAlmostEqual(center[0, 3], 0.0)
        self.assertAlmostEqual(summary["position_rms_mm"], (2.0 / 3.0) ** 0.5)
        self.assertAlmostEqual(summary["rotation_rms_deg"], 0.0)


if __name__ == "__main__":
    unittest.main()
