"""charuco 보드 감지와 camera-board TF 발행"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Return ROS quaternion (x, y, z, w) for a proper 3x3 rotation."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation must be a finite 3x3 matrix")

    # This branch-stable conversion avoids an extra scipy/tf dependency.
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.asarray([x, y, z, w], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int] | None]:
    if not path.is_file():
        raise FileNotFoundError(f"intrinsics file not found: {path}")
    with np.load(path, allow_pickle=False) as data:
        missing = {"camera_matrix", "dist_coeffs"}.difference(data.files)
        if missing:
            raise KeyError(f"intrinsics missing keys: {sorted(missing)}")
        matrix = np.asarray(data["camera_matrix"], dtype=float)
        distortion = np.asarray(data["dist_coeffs"], dtype=float)
        size = None
        if {"image_width", "image_height"}.issubset(data.files):
            size = (int(data["image_width"]), int(data["image_height"]))
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("camera_matrix must be a finite 3x3 matrix")
    if not np.isfinite(distortion).all():
        raise ValueError("dist_coeffs contains NaN or inf")
    return matrix, distortion, size


def create_charuco_detector() -> tuple[Any, Any]:
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("cv2.aruco is unavailable; install opencv-contrib-python")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    board = cv2.aruco.CharucoBoard((11, 8), 0.015, 0.011, dictionary)
    if not hasattr(board, "setLegacyPattern"):
        raise RuntimeError("OpenCV does not support CharucoBoard.setLegacyPattern")
    board.setLegacyPattern(True)
    return board, cv2.aruco.CharucoDetector(board)


class CharucoTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__("pinkk_charuco_tf_publisher")
        self.declare_parameter("camera", 0)
        self.declare_parameter("intrinsics_path", "")
        self.declare_parameter("camera_frame", "camera_optical_frame")
        self.declare_parameter("target_frame", "charuco_board")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("min_corners", 25)
        self.declare_parameter("max_reprojection_error_px", 0.7)
        self.declare_parameter("show_preview", True)

        camera_value = str(self.get_parameter("camera").value)
        camera: int | str = int(camera_value) if camera_value.isdigit() else camera_value
        self._camera_frame = str(self.get_parameter("camera_frame").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._min_corners = int(self.get_parameter("min_corners").value)
        self._max_error = float(self.get_parameter("max_reprojection_error_px").value)
        self._show_preview = bool(self.get_parameter("show_preview").value)

        path = Path(str(self.get_parameter("intrinsics_path").value)).expanduser()
        self._camera_matrix, self._dist_coeffs, expected_size = load_intrinsics(path)
        self._board, self._detector = create_charuco_detector()
        self._capture = cv2.VideoCapture(camera)
        width = int(self.get_parameter("camera_width").value)
        height = int(self.get_parameter("camera_height").value)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self._capture.isOpened():
            raise RuntimeError(f"failed to open camera: {camera_value}")
        actual_size = (
            int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
            int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        )
        if expected_size is not None and actual_size != expected_size:
            raise RuntimeError(
                f"camera size {actual_size} differs from intrinsics size {expected_size}"
            )

        self._broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(1.0 / 20.0, self._process_frame)
        self._last_detection_state: bool | None = None
        self.get_logger().info(
            f"ChArUco TF: {self._camera_frame} -> {self._target_frame}, "
            f"camera={camera_value}, size={actual_size}"
        )

    def _process_frame(self) -> None:
        ok, frame = self._capture.read()
        if not ok:
            self.get_logger().warning("camera frame read failed", throttle_duration_sec=2.0)
            return

        corners, ids, _, _ = self._detector.detectBoard(frame)
        count = 0 if ids is None else int(len(ids))
        detected = False
        error = float("inf")
        if count >= self._min_corners:
            object_points, image_points = self._board.matchImagePoints(corners, ids)
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self._camera_matrix,
                self._dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if success:
                projected, _ = cv2.projectPoints(
                    object_points, rvec, tvec, self._camera_matrix, self._dist_coeffs
                )
                residual = projected.reshape(-1, 2) - np.asarray(image_points).reshape(-1, 2)
                error = float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
                detected = math.isfinite(error) and error <= self._max_error
                if detected:
                    rotation, _ = cv2.Rodrigues(rvec)
                    self._publish_transform(rotation, np.asarray(tvec).reshape(3))

        if detected != self._last_detection_state:
            message = "DETECTED" if detected else "NOT DETECTED"
            self.get_logger().info(f"ChArUco {message}: corners={count}, error={error:.3f}px")
            self._last_detection_state = detected

        if self._show_preview:
            if corners is not None and ids is not None:
                cv2.aruco.drawDetectedCornersCharuco(frame, corners, ids)
            color = (0, 255, 0) if detected else (0, 0, 255)
            error_text = f"{error:.3f}px" if math.isfinite(error) else "--"
            cv2.putText(
                frame,
                f"Charuco: {'DETECTED' if detected else 'NOT DETECTED'} | "
                f"corners: {count} | reprojection: {error_text}",
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("charuco_tf_publisher", frame)
            cv2.waitKey(1)

    def _publish_transform(self, rotation: np.ndarray, translation: np.ndarray) -> None:
        x, y, z, w = rotation_matrix_to_quaternion(rotation)
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._camera_frame
        message.child_frame_id = self._target_frame
        message.transform.translation.x = float(translation[0])
        message.transform.translation.y = float(translation[1])
        message.transform.translation.z = float(translation[2])
        message.transform.rotation.x = x
        message.transform.rotation.y = y
        message.transform.rotation.z = z
        message.transform.rotation.w = w
        self._broadcaster.sendTransform(message)

    def close(self) -> None:
        self._capture.release()
        if self._show_preview:
            cv2.destroyAllWindows()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: CharucoTfPublisher | None = None
    try:
        node = CharucoTfPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
