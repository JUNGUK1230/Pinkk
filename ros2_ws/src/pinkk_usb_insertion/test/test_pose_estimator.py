import cv2
import numpy as np

from pinkk_usb_insertion.perception.pose_estimator import (
    estimate_port_pose,
    usb_port_object_points,
)


def test_synthetic_port_pose_depth() -> None:
    camera_matrix = np.array(
        [[1000.0, 0.0, 320.0], [0.0, 1000.0, 240.0], [0.0, 0.0, 1.0]]
    )
    distortion = np.zeros(5)
    object_points = usb_port_object_points(0.0115, 0.0045)
    image_points, _ = cv2.projectPoints(
        object_points,
        np.zeros(3),
        np.array([0.01, -0.005, 0.25]),
        camera_matrix,
        distortion,
    )
    estimate = estimate_port_pose(
        image_points.reshape(4, 2),
        camera_matrix,
        distortion,
        0.0115,
        0.0045,
    )
    assert np.isclose(estimate.depth_m, 0.25, atol=1e-6)
    assert estimate.reprojection_error_px < 1e-6
