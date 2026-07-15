"""Cartesian 관측 자세 생성, 자동 이동, 검출 확인 및 Hand-eye 수집."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory
from easy_handeye2_msgs.srv import (
    ComputeCalibration,
    SaveCalibration,
    SaveSamples,
    TakeSample,
)
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


JOINT_NAMES = (
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
)

ACTION_NAME = "/arm_group_controller/follow_joint_trajectory"
IK_SERVICE = "/compute_ik"
TAKE_SAMPLE_SERVICE = "/easy_handeye2/calibration/take_sample"
SAVE_SAMPLES_SERVICE = "/easy_handeye2/calibration/save_samples"
COMPUTE_SERVICE = "/easy_handeye2/calibration/compute_calibration"
SAVE_CALIBRATION_SERVICE = "/easy_handeye2/calibration/save_calibration"

# 홈 자세의 플랜지 위치는 유지하고 local X/Y/Z 방향을 다양하게 회전한다.
# 단위는 degree이며 (roll, pitch, yaw) 순서다.
OBSERVATION_ROTATIONS_DEG = (
    (0.0, 0.0, 0.0),
    (12.0, 0.0, 0.0),
    (-12.0, 0.0, 0.0),
    (0.0, 12.0, 0.0),
    (0.0, -12.0, 0.0),
    (0.0, 0.0, 15.0),
    (0.0, 0.0, -15.0),
    (10.0, 10.0, 0.0),
    (-10.0, -10.0, 0.0),
    (10.0, -10.0, 0.0),
    (-10.0, 10.0, 0.0),
    (10.0, 0.0, 12.0),
    (-10.0, 0.0, -12.0),
    (0.0, 10.0, -12.0),
    (0.0, -10.0, 12.0),
)


def quaternion_multiply(a: Sequence[float], b: Sequence[float]) -> tuple[float, ...]:
    ax, ay, az, aw = (float(value) for value in a)
    bx, by, bz, bw = (float(value) for value in b)
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_from_rpy_degrees(roll: float, pitch: float, yaw: float) -> tuple[float, ...]:
    roll, pitch, yaw = (math.radians(value) / 2.0 for value in (roll, pitch, yaw))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class AutoHandeyeCollector(Node):
    def __init__(self) -> None:
        super().__init__("pinkk_handeye_auto_collect")
        self.declare_parameter("execute", False)
        self.declare_parameter("settle_seconds", 1.5)
        self.declare_parameter("detection_timeout_seconds", 8.0)
        self.declare_parameter("max_tf_age_seconds", 0.4)
        self.declare_parameter("motion_seconds", 4.0)
        self.declare_parameter("motion_retry_count", 2)
        self.declare_parameter("return_home", True)

        self.execute_enabled = bool(self.get_parameter("execute").value)
        self.settle_seconds = float(self.get_parameter("settle_seconds").value)
        self.detection_timeout = float(
            self.get_parameter("detection_timeout_seconds").value
        )
        self.max_tf_age = float(self.get_parameter("max_tf_age_seconds").value)
        self.motion_seconds = float(self.get_parameter("motion_seconds").value)
        self.motion_retry_count = int(self.get_parameter("motion_retry_count").value)
        self.return_home = bool(self.get_parameter("return_home").value)

        self._state_lock = threading.Lock()
        self._latest_joints: list[float] | None = None
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0), node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._action = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self._ik = self.create_client(GetPositionIK, IK_SERVICE)
        self._take_sample = self.create_client(TakeSample, TAKE_SAMPLE_SERVICE)
        self._save_samples = self.create_client(SaveSamples, SAVE_SAMPLES_SERVICE)
        self._compute = self.create_client(ComputeCalibration, COMPUTE_SERVICE)
        self._save_calibration = self.create_client(
            SaveCalibration, SAVE_CALIBRATION_SERVICE
        )

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
        self.get_logger().info("필요한 action/service/TF를 확인합니다")
        if not self._action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"action server가 없습니다: {ACTION_NAME}")
        for client, name in (
            (self._ik, IK_SERVICE),
            (self._take_sample, TAKE_SAMPLE_SERVICE),
            (self._save_samples, SAVE_SAMPLES_SERVICE),
            (self._compute, COMPUTE_SERVICE),
            (self._save_calibration, SAVE_CALIBRATION_SERVICE),
        ):
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"service가 없습니다: {name}")

        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            with self._state_lock:
                if self._latest_joints is not None:
                    return list(self._latest_joints)
            time.sleep(0.05)
        raise RuntimeError("/joint_states를 받지 못했습니다")

    def _home_pose(self) -> tuple[PoseStamped, list[float]]:
        transform = self._tf_buffer.lookup_transform(
            "g_base", "joint6_flange", Time(), Duration(seconds=3.0)
        )
        pose = PoseStamped()
        pose.header.frame_id = "g_base"
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        with self._state_lock:
            joints = list(self._latest_joints or [])
        if len(joints) != 6:
            raise RuntimeError("홈 관절각을 읽지 못했습니다")
        return pose, joints

    def _target_pose(self, home: PoseStamped, rpy: Sequence[float]) -> PoseStamped:
        target = PoseStamped()
        target.header.frame_id = "g_base"
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position = home.pose.position
        base_q = (
            home.pose.orientation.x,
            home.pose.orientation.y,
            home.pose.orientation.z,
            home.pose.orientation.w,
        )
        delta_q = quaternion_from_rpy_degrees(*rpy)
        x, y, z, w = quaternion_multiply(base_q, delta_q)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        target.pose.orientation = Quaternion(x=x / norm, y=y / norm, z=z / norm, w=w / norm)
        return target

    def _solve_ik(self, target: PoseStamped, seed: Sequence[float]) -> list[float] | None:
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm_group"
        request.ik_request.ik_link_name = "joint6_flange"
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

    def _solve_observation_ik(
        self,
        home: PoseStamped,
        rotation: Sequence[float],
        seed: Sequence[float],
    ) -> tuple[list[float] | None, tuple[float, float, float]]:
        """원래 회전이 불가능하면 각도를 줄여 같은 방향으로 IK를 재시도한다."""
        for scale in (1.0, 0.75, 0.5, 0.35):
            adjusted = tuple(round(float(value) * scale, 3) for value in rotation)
            target = self._target_pose(home, adjusted)
            solution = self._solve_ik(target, seed)
            if solution is not None:
                return solution, adjusted
        return None, tuple(float(value) for value in rotation)

    @staticmethod
    def _scaled_rotations(
        rotation: Sequence[float],
    ) -> tuple[tuple[float, float, float], ...]:
        """검출이나 이동이 실패할 때 같은 방향의 더 작은 관측각을 만든다."""
        candidates: list[tuple[float, float, float]] = []
        for scale in (1.0, 0.75, 0.5, 0.35):
            adjusted = tuple(round(float(value) * scale, 3) for value in rotation)
            if adjusted not in candidates:
                candidates.append(adjusted)
        return tuple(candidates)

    def _current_joint_seed(self, fallback: Sequence[float]) -> list[float]:
        with self._state_lock:
            current = list(self._latest_joints or [])
        return current if len(current) == 6 else list(fallback)

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
                    wrapped_result = self._wait_future(
                        handle.get_result_async(), self.motion_seconds + 15.0
                    )
                    if (
                        wrapped_result.result.error_code
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

    def _wait_valid_detection(self) -> bool:
        deadline = time.monotonic() + self.detection_timeout
        stable = 0
        while time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    "camera_optical_frame", "charuco_board", Time()
                )
                stamp = Time.from_msg(transform.header.stamp)
                age = abs((self.get_clock().now() - stamp).nanoseconds * 1e-9)
                values = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                )
                valid = (
                    age <= self.max_tf_age
                    and transform.transform.translation.z > 0.0
                    and all(math.isfinite(value) for value in values)
                )
            except TransformException:
                valid = False
            stable = stable + 1 if valid else 0
            if stable >= 5:
                return True
            time.sleep(0.1)
        return False

    def run(self) -> None:
        home_joints = self._wait_ready()
        home_pose, home_joints = self._home_pose()
        if not self.execute_enabled:
            self.get_logger().warning(
                "DRY RUN입니다. 실제 자동 수집은 execute:=true로 실행하세요"
            )
            ik_successes = 0
            for index, rotation in enumerate(OBSERVATION_ROTATIONS_DEG, start=1):
                solution, adjusted = self._solve_observation_ik(
                    home_pose, rotation, home_joints
                )
                status = "IK OK" if solution is not None else "IK 실패"
                ik_successes += int(solution is not None)
                self.get_logger().info(
                    f"자세 {index:02d} 요청={rotation}, 적용={adjusted}: {status}"
                )
            self.get_logger().info(
                f"DRY RUN 완료: IK 성공 {ik_successes}/{len(OBSERVATION_ROTATIONS_DEG)}"
            )
            return

        self.get_logger().warning("5초 후 자동 이동과 샘플 수집을 시작합니다")
        time.sleep(5.0)
        collected = 0
        seed = home_joints
        try:
            for index, rotation in enumerate(OBSERVATION_ROTATIONS_DEG, start=1):
                self.get_logger().info(
                    f"[{index}/{len(OBSERVATION_ROTATIONS_DEG)}] 목표 회전 [deg]: {rotation}"
                )
                pose_collected = False
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
                    if adjusted != tuple(rotation):
                        self.get_logger().info(
                            f"축소 관측 회전으로 재시도 [deg]: {adjusted}"
                        )
                    if not self._move(solution):
                        self.get_logger().warning(
                            f"이동 실패, 관측각 축소 재시도: {adjusted}"
                        )
                        continue
                    seed = solution
                    time.sleep(self.settle_seconds)
                    if not self._wait_valid_detection():
                        self.get_logger().warning(
                            f"ChArUco 검출 실패, 관측각 축소 재시도: {adjusted}"
                        )
                        continue
                    response = self._wait_future(
                        self._take_sample.call_async(TakeSample.Request()), 5.0
                    )
                    new_count = len(response.samples.samples)
                    if new_count <= collected:
                        self.get_logger().warning(
                            f"Easy Handeye 샘플 추가 실패, 재시도: {adjusted}"
                        )
                        continue
                    collected = new_count
                    pose_collected = True
                    self.get_logger().info(f"샘플 저장 완료: {collected}개")
                    break
                if not pose_collected:
                    self.get_logger().warning(
                        f"자세 {index:02d}의 모든 축소 관측 시도가 실패했습니다"
                    )
        finally:
            if self.return_home:
                self.get_logger().info("초기 홈 자세로 복귀합니다")
                self._move(home_joints)

        if collected < 10:
            raise RuntimeError(f"유효 샘플이 {collected}개뿐이라 계산하지 않습니다")
        self._wait_future(self._save_samples.call_async(SaveSamples.Request()), 5.0)
        computed = self._wait_future(
            self._compute.call_async(ComputeCalibration.Request()), 20.0
        )
        if not computed.valid:
            raise RuntimeError("Easy Handeye 캘리브레이션 계산에 실패했습니다")
        saved = self._wait_future(
            self._save_calibration.call_async(SaveCalibration.Request()), 5.0
        )
        if not saved.success:
            raise RuntimeError("캘리브레이션 결과 저장에 실패했습니다")
        self.get_logger().info(
            f"자동 캘리브레이션 완료: samples={collected}, file={saved.filepath.data}"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = AutoHandeyeCollector()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warning("사용자가 자동 수집을 중단했습니다")
    except Exception as error:
        node.get_logger().error(f"자동 수집 실패: {error}")
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
