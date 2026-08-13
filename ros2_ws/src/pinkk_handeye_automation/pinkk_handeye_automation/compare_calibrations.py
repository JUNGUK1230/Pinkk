"""Compare two eye-in-hand calibrations over the same automatic robot poses."""

from __future__ import annotations

import csv
import json
import math
import threading
import time
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from .auto_collect import (
    ACTION_NAME,
    IK_SERVICE,
    JOINT_NAMES,
    OBSERVATION_ROTATIONS_DEG,
    quaternion_from_rpy_degrees,
    quaternion_multiply,
)


BASE_FRAME = "g_base"
EFFECTOR_FRAME = "joint6_flange"
CAMERA_FRAME = "camera_optical_frame"
BOARD_FRAME = "charuco_board"


def quaternion_to_matrix(quaternion: Sequence[float]) -> np.ndarray:
    """Convert an x/y/z/w quaternion to a 3x3 rotation matrix."""
    values = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12 or not np.isfinite(values).all():
        raise ValueError("유효하지 않은 quaternion입니다")
    x, y, z, w = values / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to a normalized x/y/z/w quaternion."""
    matrix = np.asarray(rotation, dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = (
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = (
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = (
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            )
    values = np.asarray(quaternion, dtype=float)
    values /= np.linalg.norm(values)
    if values[3] < 0.0:
        values *= -1.0
    return tuple(float(value) for value in values)


def transform_matrix(
    translation: Sequence[float], quaternion: Sequence[float]
) -> np.ndarray:
    result = np.eye(4, dtype=float)
    result[:3, :3] = quaternion_to_matrix(quaternion)
    result[:3, 3] = np.asarray(translation, dtype=float)
    return result


def transform_message_to_matrix(message: object) -> np.ndarray:
    return transform_matrix(
        (
            message.translation.x,
            message.translation.y,
            message.translation.z,
        ),
        (
            message.rotation.x,
            message.rotation.y,
            message.rotation.z,
            message.rotation.w,
        ),
    )


def rotation_angle_degrees(rotation: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def matrix_to_rpy_degrees(rotation: np.ndarray) -> tuple[float, float, float]:
    pitch = math.atan2(
        -float(rotation[2, 0]),
        math.hypot(float(rotation[0, 0]), float(rotation[1, 0])),
    )
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = math.atan2(-float(rotation[1, 2]), float(rotation[1, 1]))
        yaw = 0.0
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def mean_transform(transforms: Sequence[np.ndarray]) -> np.ndarray:
    if not transforms:
        raise ValueError("평균을 계산할 transform이 없습니다")
    result = np.eye(4, dtype=float)
    result[:3, 3] = np.mean([matrix[:3, 3] for matrix in transforms], axis=0)
    mean_rotation = np.mean([matrix[:3, :3] for matrix in transforms], axis=0)
    left, _, right = np.linalg.svd(mean_rotation)
    rotation = left @ right
    if np.linalg.det(rotation) < 0.0:
        left[:, -1] *= -1.0
        rotation = left @ right
    result[:3, :3] = rotation
    return result


def summarize_transforms(transforms: Sequence[np.ndarray]) -> dict[str, object]:
    center = mean_transform(transforms)
    position_errors = [
        float(np.linalg.norm(matrix[:3, 3] - center[:3, 3]) * 1000.0)
        for matrix in transforms
    ]
    rotation_errors = [
        rotation_angle_degrees(center[:3, :3].T @ matrix[:3, :3])
        for matrix in transforms
    ]
    quaternion = matrix_to_quaternion(center[:3, :3])
    rpy = matrix_to_rpy_degrees(center[:3, :3])
    return {
        "pose_count": len(transforms),
        "mean_translation_m": {
            axis: float(center[index, 3]) for index, axis in enumerate("xyz")
        },
        "mean_quaternion_xyzw": {
            axis: quaternion[index] for index, axis in enumerate("xyzw")
        },
        "mean_rpy_deg": {axis: rpy[index] for index, axis in enumerate(("roll", "pitch", "yaw"))},
        "position_rms_mm": float(math.sqrt(np.mean(np.square(position_errors)))),
        "position_max_mm": float(max(position_errors)),
        "rotation_rms_deg": float(math.sqrt(np.mean(np.square(rotation_errors)))),
        "rotation_max_deg": float(max(rotation_errors)),
    }


def load_calibration(path: Path) -> tuple[np.ndarray, dict[str, object]]:
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"올바른 YAML calibration 파일이 아닙니다: {path}")
    parameters = data.get("parameters", {})
    expected = {
        "calibration_type": "eye_in_hand",
        "robot_base_frame": BASE_FRAME,
        "robot_effector_frame": EFFECTOR_FRAME,
        "tracking_base_frame": CAMERA_FRAME,
        "tracking_marker_frame": BOARD_FRAME,
    }
    mismatches = [
        f"{key}={parameters.get(key)!r} (expected {value!r})"
        for key, value in expected.items()
        if parameters.get(key) != value
    ]
    if mismatches:
        raise ValueError(f"{path} frame/type 불일치: {', '.join(mismatches)}")
    try:
        translation_data = data["transform"]["translation"]
        rotation_data = data["transform"]["rotation"]
        translation = tuple(float(translation_data[axis]) for axis in "xyz")
        quaternion = tuple(float(rotation_data[axis]) for axis in "xyzw")
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{path} transform 형식이 올바르지 않습니다") from error
    matrix = transform_matrix(translation, quaternion)
    if not np.isfinite(matrix).all():
        raise ValueError(f"{path} transform에 유한하지 않은 값이 있습니다")
    return matrix, parameters


class CalibrationComparator(Node):
    def __init__(self) -> None:
        super().__init__("pinkk_handeye_compare_calibrations")
        self.declare_parameter("execute", False)
        self.declare_parameter("old_calib_path", "")
        self.declare_parameter("new_calib_path", "")
        self.declare_parameter("output_csv", "")
        self.declare_parameter("pose_limit", 30)
        self.declare_parameter("target_valid_poses", 0)
        self.declare_parameter("settle_seconds", 1.5)
        self.declare_parameter("detection_timeout_seconds", 8.0)
        self.declare_parameter("max_tf_age_seconds", 0.4)
        self.declare_parameter("tf_sync_delay_seconds", 0.2)
        self.declare_parameter("motion_seconds", 4.0)
        self.declare_parameter("motion_retry_count", 2)
        self.declare_parameter("measurement_count", 10)
        self.declare_parameter("measurement_interval_seconds", 0.1)
        self.declare_parameter("return_home", True)

        self.execute_enabled = bool(self.get_parameter("execute").value)
        self.old_path = Path(
            str(self.get_parameter("old_calib_path").value)
        ).expanduser()
        self.new_path = Path(
            str(self.get_parameter("new_calib_path").value)
        ).expanduser()
        output_value = str(self.get_parameter("output_csv").value)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_csv = (
            Path(output_value).expanduser()
            if output_value
            else Path.home() / f"handeye_comparison_{timestamp}.csv"
        )
        self.pose_limit = int(self.get_parameter("pose_limit").value)
        configured_target = int(
            self.get_parameter("target_valid_poses").value
        )
        self.target_valid_poses = configured_target or self.pose_limit
        self.settle_seconds = float(self.get_parameter("settle_seconds").value)
        self.detection_timeout = float(
            self.get_parameter("detection_timeout_seconds").value
        )
        self.max_tf_age = float(self.get_parameter("max_tf_age_seconds").value)
        self.tf_sync_delay = float(
            self.get_parameter("tf_sync_delay_seconds").value
        )
        self.motion_seconds = float(self.get_parameter("motion_seconds").value)
        self.motion_retry_count = int(self.get_parameter("motion_retry_count").value)
        self.measurement_count = int(self.get_parameter("measurement_count").value)
        self.measurement_interval = float(
            self.get_parameter("measurement_interval_seconds").value
        )
        self.return_home = bool(self.get_parameter("return_home").value)
        if not 1 <= self.pose_limit <= len(OBSERVATION_ROTATIONS_DEG):
            raise ValueError(
                f"pose_limit은 1~{len(OBSERVATION_ROTATIONS_DEG)} 범위여야 합니다"
            )
        if not 1 <= self.target_valid_poses <= self.pose_limit:
            raise ValueError(
                "target_valid_poses는 1 이상 pose_limit 이하여야 합니다"
            )
        if self.measurement_count < 1:
            raise ValueError("measurement_count는 1 이상이어야 합니다")
        if not 0.0 < self.tf_sync_delay < self.max_tf_age:
            raise ValueError(
                "tf_sync_delay_seconds는 0보다 크고 "
                "max_tf_age_seconds보다 작아야 합니다"
            )
        if not self.old_path.is_file() or not self.new_path.is_file():
            raise FileNotFoundError(
                f"calibration 파일을 확인하세요: old={self.old_path}, new={self.new_path}"
            )
        self.old_calibration, self.old_parameters = load_calibration(self.old_path)
        self.new_calibration, self.new_parameters = load_calibration(self.new_path)

        self._state_lock = threading.Lock()
        self._latest_joints: list[float] | None = None
        self._last_tf_rejection_log = 0.0
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0), node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._action = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self._ik = self.create_client(GetPositionIK, IK_SERVICE)

    def _joint_callback(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in JOINT_NAMES):
            return
        ordered = [float(values[name]) for name in JOINT_NAMES]
        if not all(math.isfinite(value) for value in ordered):
            return
        with self._state_lock:
            self._latest_joints = ordered

    def _wait_future(self, future: object, timeout: float) -> object:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError("ROS 응답 대기 시간이 초과됐습니다")
            time.sleep(0.05)
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def _wait_ready(self) -> list[float]:
        self.get_logger().info("trajectory action, MoveIt IK, joint_states를 확인합니다")
        if not self._action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"action server가 없습니다: {ACTION_NAME}")
        if not self._ik.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"service가 없습니다: {IK_SERVICE}")
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            with self._state_lock:
                if self._latest_joints is not None:
                    return list(self._latest_joints)
            time.sleep(0.05)
        raise RuntimeError("/joint_states를 받지 못했습니다")

    def _home_pose(self) -> tuple[PoseStamped, list[float]]:
        transform = self._tf_buffer.lookup_transform(
            BASE_FRAME, EFFECTOR_FRAME, Time(), Duration(seconds=3.0)
        )
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        with self._state_lock:
            joints = list(self._latest_joints or [])
        if len(joints) != len(JOINT_NAMES):
            raise RuntimeError("홈 관절각을 읽지 못했습니다")
        return pose, joints

    def _target_pose(self, home: PoseStamped, rpy: Sequence[float]) -> PoseStamped:
        target = PoseStamped()
        target.header.frame_id = BASE_FRAME
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position = home.pose.position
        base = (
            home.pose.orientation.x,
            home.pose.orientation.y,
            home.pose.orientation.z,
            home.pose.orientation.w,
        )
        result = quaternion_multiply(base, quaternion_from_rpy_degrees(*rpy))
        norm = math.sqrt(sum(value * value for value in result))
        target.pose.orientation = Quaternion(
            x=result[0] / norm,
            y=result[1] / norm,
            z=result[2] / norm,
            w=result[3] / norm,
        )
        return target

    def _solve_ik(
        self, target: PoseStamped, seed: Sequence[float]
    ) -> list[float] | None:
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm_group"
        request.ik_request.ik_link_name = EFFECTOR_FRAME
        request.ik_request.pose_stamped = target
        request.ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.ik_request.robot_state.joint_state.position = list(seed)
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=2.0).to_msg()
        response = self._wait_future(self._ik.call_async(request), 5.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        values = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        if not all(name in values for name in JOINT_NAMES):
            return None
        return [float(values[name]) for name in JOINT_NAMES]

    @staticmethod
    def _scaled_rotations(
        rotation: Sequence[float],
    ) -> tuple[tuple[float, float, float], ...]:
        candidates: list[tuple[float, float, float]] = []
        for scale in (1.0, 0.75, 0.5, 0.35):
            adjusted = tuple(round(float(value) * scale, 3) for value in rotation)
            if adjusted not in candidates:
                candidates.append(adjusted)
        return tuple(candidates)

    def _current_joint_seed(self, fallback: Sequence[float]) -> list[float]:
        with self._state_lock:
            current = list(self._latest_joints or [])
        return current if len(current) == len(JOINT_NAMES) else list(fallback)

    def _move(self, target: Sequence[float]) -> bool:
        for attempt in range(1, max(1, self.motion_retry_count) + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINT_NAMES)
            point = JointTrajectoryPoint()
            point.positions = list(target)
            point.time_from_start = Duration(seconds=self.motion_seconds).to_msg()
            goal.trajectory.points = [point]
            try:
                handle = self._wait_future(self._action.send_goal_async(goal), 5.0)
                if handle.accepted:
                    wrapped = self._wait_future(
                        handle.get_result_async(), self.motion_seconds + 15.0
                    )
                    if (
                        wrapped.result.error_code
                        == FollowJointTrajectory.Result.SUCCESSFUL
                    ):
                        return True
            except (RuntimeError, TimeoutError) as error:
                self.get_logger().warning(
                    f"이동 시도 {attempt} 응답 실패: {error}"
                )
            if attempt < max(1, self.motion_retry_count):
                self.get_logger().warning(
                    f"동일 목표 이동 재시도: {attempt + 1}/{self.motion_retry_count}"
                )
                time.sleep(0.5)
        return False

    def _raw_transform_pair(
        self,
    ) -> tuple[np.ndarray, np.ndarray, int] | None:
        # The camera TF is published faster than the robot TF. Looking up the
        # robot chain at the newest camera stamp therefore frequently asks TF2
        # to extrapolate into the future. Query both chains at the same,
        # slightly delayed time so that both samples are already buffered.
        sample_time = self.get_clock().now() - Duration(
            seconds=self.tf_sync_delay
        )
        try:
            camera_board = self._tf_buffer.lookup_transform(
                CAMERA_FRAME, BOARD_FRAME, sample_time
            )
            base_flange = self._tf_buffer.lookup_transform(
                BASE_FRAME, EFFECTOR_FRAME, sample_time
            )
        except TransformException as error:
            now = time.monotonic()
            if now - self._last_tf_rejection_log >= 2.0:
                self._last_tf_rejection_log = now
                self.get_logger().warning(
                    "동일 시각 TF 조회 실패 "
                    f"(delay={self.tf_sync_delay:.3f}s): {error}"
                )
            return None
        stamp = Time.from_msg(camera_board.header.stamp)
        age = abs((self.get_clock().now() - stamp).nanoseconds * 1e-9)
        camera_values = (
            camera_board.transform.translation.x,
            camera_board.transform.translation.y,
            camera_board.transform.translation.z,
            camera_board.transform.rotation.x,
            camera_board.transform.rotation.y,
            camera_board.transform.rotation.z,
            camera_board.transform.rotation.w,
        )
        if (
            age > self.max_tf_age
            or camera_board.transform.translation.z <= 0.0
            or not all(math.isfinite(value) for value in camera_values)
        ):
            now = time.monotonic()
            if now - self._last_tf_rejection_log >= 2.0:
                self._last_tf_rejection_log = now
                self.get_logger().warning(
                    "TF 측정값 거부: "
                    f"age={age:.3f}s/{self.max_tf_age:.3f}s, "
                    f"board_z={camera_board.transform.translation.z:.4f}m"
                )
            return None
        return (
            transform_message_to_matrix(base_flange.transform),
            transform_message_to_matrix(camera_board.transform),
            stamp.nanoseconds,
        )

    def _collect_pose_measurements(
        self,
    ) -> tuple[list[np.ndarray], list[np.ndarray]]:
        old_values: list[np.ndarray] = []
        new_values: list[np.ndarray] = []
        stable = 0
        last_stamp: int | None = None
        deadline = time.monotonic() + self.detection_timeout
        while time.monotonic() < deadline and len(old_values) < self.measurement_count:
            pair = self._raw_transform_pair()
            stable = stable + 1 if pair is not None else 0
            if pair is not None and stable >= 5:
                base_flange, camera_board, stamp = pair
                if stamp == last_stamp:
                    time.sleep(self.measurement_interval)
                    continue
                last_stamp = stamp
                old_values.append(
                    base_flange @ self.old_calibration @ camera_board
                )
                new_values.append(
                    base_flange @ self.new_calibration @ camera_board
                )
            time.sleep(self.measurement_interval)
        return old_values, new_values

    @staticmethod
    def _pose_row(
        index: int,
        requested: Sequence[float],
        applied: Sequence[float],
        candidate: str,
        calibration_path: Path,
        transforms: Sequence[np.ndarray],
    ) -> dict[str, object]:
        summary = summarize_transforms(transforms)
        mean = mean_transform(transforms)
        quaternion = matrix_to_quaternion(mean[:3, :3])
        rpy = matrix_to_rpy_degrees(mean[:3, :3])
        row: dict[str, object] = {
            "pose_index": index,
            "requested_roll_deg": requested[0],
            "requested_pitch_deg": requested[1],
            "requested_yaw_deg": requested[2],
            "applied_roll_deg": applied[0],
            "applied_pitch_deg": applied[1],
            "applied_yaw_deg": applied[2],
            "candidate": candidate,
            "calibration_path": str(calibration_path),
            "measurement_count": len(transforms),
            "board_x_m": float(mean[0, 3]),
            "board_y_m": float(mean[1, 3]),
            "board_z_m": float(mean[2, 3]),
            "board_qx": quaternion[0],
            "board_qy": quaternion[1],
            "board_qz": quaternion[2],
            "board_qw": quaternion[3],
            "board_roll_deg": rpy[0],
            "board_pitch_deg": rpy[1],
            "board_yaw_deg": rpy[2],
            "within_position_rms_mm": summary["position_rms_mm"],
            "within_rotation_rms_deg": summary["rotation_rms_deg"],
        }
        return row

    def _write_results(
        self,
        rows: Sequence[dict[str, object]],
        old_pose_values: Sequence[np.ndarray],
        new_pose_values: Sequence[np.ndarray],
    ) -> tuple[Path, Path]:
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        with self.output_csv.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

        delta = np.linalg.inv(self.old_calibration) @ self.new_calibration
        summary = {
            "created_at": datetime.now().astimezone().isoformat(),
            "frames": {
                "robot_base": BASE_FRAME,
                "robot_effector": EFFECTOR_FRAME,
                "tracking_base": CAMERA_FRAME,
                "tracking_marker": BOARD_FRAME,
            },
            "old": {
                "calibration_path": str(self.old_path),
                **summarize_transforms(old_pose_values),
            },
            "new": {
                "calibration_path": str(self.new_path),
                **summarize_transforms(new_pose_values),
            },
            "calibration_transform_difference": {
                "translation_mm": float(np.linalg.norm(delta[:3, 3]) * 1000.0),
                "rotation_deg": rotation_angle_degrees(delta[:3, :3]),
            },
            "interpretation": (
                "고정된 보드의 g_base 기준 자세 산포가 작을수록 일관성이 좋은 "
                "캘리브레이션입니다."
            ),
        }
        summary_path = self.output_csv.with_suffix(".summary.json")
        with summary_path.open("w", encoding="utf-8") as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        return self.output_csv, summary_path

    def _log_calibration_difference(self) -> None:
        delta = np.linalg.inv(self.old_calibration) @ self.new_calibration
        self.get_logger().info(
            "두 calibration 행렬 차이: "
            f"translation={np.linalg.norm(delta[:3, 3]) * 1000.0:.2f} mm, "
            f"rotation={rotation_angle_degrees(delta[:3, :3]):.2f} deg"
        )

    def run(self) -> None:
        home_joints = self._wait_ready()
        home_pose, home_joints = self._home_pose()
        rotations = OBSERVATION_ROTATIONS_DEG[: self.pose_limit]
        self._log_calibration_difference()

        if not self.execute_enabled:
            self.get_logger().warning(
                "CHECK 모드입니다. 로봇은 움직이지 않고 모든 자세의 IK만 검사합니다"
            )
            successes = 0
            for index, rotation in enumerate(rotations, start=1):
                solution = None
                applied = rotation
                for adjusted in self._scaled_rotations(rotation):
                    solution = self._solve_ik(
                        self._target_pose(home_pose, adjusted), home_joints
                    )
                    if solution is not None:
                        applied = adjusted
                        break
                successes += int(solution is not None)
                status = "IK OK" if solution is not None else "IK 실패"
                self.get_logger().info(
                    f"자세 {index:02d} 요청={rotation}, 적용={applied}: {status}"
                )
            self.get_logger().info(
                f"CHECK 완료: IK 성공 {successes}/{len(rotations)}. "
                "이 모드에서는 CSV를 만들지 않습니다"
            )
            return

        self.get_logger().warning(
            "5초 후 실제 비교 이동을 시작합니다. ChArUco 보드는 완전히 고정하세요. "
            f"목표 유효 자세={self.target_valid_poses}, "
            f"자세당 측정={self.measurement_count}회"
        )
        time.sleep(5.0)
        seed = home_joints
        rows: list[dict[str, object]] = []
        old_pose_values: list[np.ndarray] = []
        new_pose_values: list[np.ndarray] = []
        try:
            for index, rotation in enumerate(rotations, start=1):
                self.get_logger().info(
                    f"[{index}/{len(rotations)}] 비교 자세 [deg]: {rotation}"
                )
                measured = False
                for adjusted in self._scaled_rotations(rotation):
                    seed = self._current_joint_seed(seed)
                    solution = self._solve_ik(
                        self._target_pose(home_pose, adjusted), seed
                    )
                    if solution is None:
                        self.get_logger().warning(
                            f"IK 실패, 관측각 축소 재시도: {adjusted}"
                        )
                        continue
                    if not self._move(solution):
                        self.get_logger().warning(
                            f"이동 실패, 관측각 축소 재시도: {adjusted}"
                        )
                        continue
                    seed = solution
                    time.sleep(self.settle_seconds)
                    old_measurements, new_measurements = (
                        self._collect_pose_measurements()
                    )
                    if len(old_measurements) < self.measurement_count:
                        self.get_logger().warning(
                            "ChArUco 연속 측정 부족 "
                            f"({len(old_measurements)}/{self.measurement_count}), "
                            f"관측각 축소 재시도: {adjusted}"
                        )
                        continue
                    old_mean = mean_transform(old_measurements)
                    new_mean = mean_transform(new_measurements)
                    old_pose_values.append(old_mean)
                    new_pose_values.append(new_mean)
                    rows.extend(
                        (
                            self._pose_row(
                                index,
                                rotation,
                                adjusted,
                                "old",
                                self.old_path,
                                old_measurements,
                            ),
                            self._pose_row(
                                index,
                                rotation,
                                adjusted,
                                "new",
                                self.new_path,
                                new_measurements,
                            ),
                        )
                    )
                    measured = True
                    self.get_logger().info(
                        f"자세 {index:02d} 측정 완료: "
                        f"{len(old_measurements)}회 × old/new "
                        f"(유효 {len(old_pose_values)}/{self.target_valid_poses})"
                    )
                    break
                if not measured:
                    self.get_logger().warning(
                        f"자세 {index:02d}의 모든 축소 관측 시도가 실패했습니다"
                    )
                if len(old_pose_values) >= self.target_valid_poses:
                    self.get_logger().info(
                        f"목표 유효 자세 {self.target_valid_poses}개를 채워 "
                        "남은 후보를 시도하지 않습니다"
                    )
                    break
        finally:
            if self.return_home:
                self.get_logger().info("초기 홈 자세로 복귀합니다")
                self._move(home_joints)

        if len(old_pose_values) < 3:
            raise RuntimeError(
                f"유효 비교 자세가 {len(old_pose_values)}개뿐입니다. "
                "최소 3개가 필요하며 결과 파일은 저장하지 않습니다"
            )
        csv_path, summary_path = self._write_results(
            rows, old_pose_values, new_pose_values
        )
        old_summary = summarize_transforms(old_pose_values)
        new_summary = summarize_transforms(new_pose_values)
        self.get_logger().info(
            "비교 완료 "
            f"(유효 자세 {len(old_pose_values)}/{self.target_valid_poses}개): "
            f"OLD position={old_summary['position_rms_mm']:.2f} mm, "
            f"rotation={old_summary['rotation_rms_deg']:.2f} deg / "
            f"NEW position={new_summary['position_rms_mm']:.2f} mm, "
            f"rotation={new_summary['rotation_rms_deg']:.2f} deg"
        )
        self.get_logger().info(f"CSV: {csv_path}")
        self.get_logger().info(f"요약 JSON: {summary_path}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CalibrationComparator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warning("사용자가 calibration 비교를 중단했습니다")
    except Exception as error:
        node.get_logger().error(f"calibration 비교 실패: {error}")
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
