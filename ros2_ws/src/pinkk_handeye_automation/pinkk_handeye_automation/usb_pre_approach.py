"""Hand-eye/SolvePnP 좌표 정확도 검증을 위해 USB 근처로 시험 이동한다.

이 모듈은 그리퍼 TCP, 충돌 회피, 힘 제어, PBVS/IBVS가 완성되기 전 단계에서
측정한 ``g_base -> usb_port``가 실제 로봇 위치와 어느 정도 일치하는지 확인하는
실험 도구다. 생산용 USB 삽입 제어기로 사용하지 않는다.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import rclpy
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
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
DEFAULT_REFERENCE = Path.home() / ".ros2" / "pinkk_usb_pre_reference.json"

# USB를 처음 관측할 때 사용하는 사용자 지정 관절 자세 [degree].
OBSERVATION_JOINTS_DEG = (-1.66, -8.08, -36.65, -39.9, 0.0, 45.0)


@dataclass(frozen=True)
class RigidTransform:
    """parent 좌표계에서 본 child 자세. quaternion 순서는 xyzw다."""

    translation: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


@dataclass(frozen=True)
class Waypoint:
    stage: str
    transform: RigidTransform


def _normalize_quaternion(values: Sequence[float]) -> tuple[float, float, float, float]:
    x, y, z, w = (float(value) for value in values)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("길이가 0인 quaternion입니다")
    return x / norm, y / norm, z / norm, w / norm


def _quaternion_multiply(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return _normalize_quaternion(
        (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )
    )


def _rotate_vector(
    quaternion: Sequence[float], vector: Sequence[float]
) -> tuple[float, float, float]:
    x, y, z, w = _normalize_quaternion(quaternion)
    vx, vy, vz = (float(value) for value in vector)
    # v' = v + 2*w*(q_xyz x v) + 2*(q_xyz x (q_xyz x v))
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def compose(left: RigidTransform, right: RigidTransform) -> RigidTransform:
    """T_a_c = T_a_b @ T_b_c."""
    rotated = _rotate_vector(left.quaternion, right.translation)
    return RigidTransform(
        tuple(a + b for a, b in zip(left.translation, rotated)),
        _quaternion_multiply(left.quaternion, right.quaternion),
    )


def inverse(transform: RigidTransform) -> RigidTransform:
    x, y, z, w = _normalize_quaternion(transform.quaternion)
    inverse_q = (-x, -y, -z, w)
    inverse_t = _rotate_vector(
        inverse_q, tuple(-value for value in transform.translation)
    )
    return RigidTransform(inverse_t, inverse_q)


def _slerp(
    start: Sequence[float], end: Sequence[float], ratio: float
) -> tuple[float, float, float, float]:
    first = _normalize_quaternion(start)
    second = _normalize_quaternion(end)
    dot = sum(a * b for a, b in zip(first, second))
    if dot < 0.0:
        second = tuple(-value for value in second)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return _normalize_quaternion(
            tuple(a + ratio * (b - a) for a, b in zip(first, second))
        )
    theta = math.acos(dot)
    denominator = math.sin(theta)
    first_weight = math.sin((1.0 - ratio) * theta) / denominator
    second_weight = math.sin(ratio * theta) / denominator
    return _normalize_quaternion(
        tuple(
            first_weight * a + second_weight * b
            for a, b in zip(first, second)
        )
    )


def _quaternion_angle_degrees(start: Sequence[float], end: Sequence[float]) -> float:
    dot = abs(sum(a * b for a, b in zip(
        _normalize_quaternion(start), _normalize_quaternion(end)
    )))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def _quaternion_from_rpy_degrees(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    roll, pitch, yaw = (
        math.radians(value) * 0.5 for value in (roll, pitch, yaw)
    )
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return _normalize_quaternion(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def _yaw_degrees(quaternion: Sequence[float]) -> float:
    x, y, z, w = _normalize_quaternion(quaternion)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(sin_yaw, cos_yaw))


def make_waypoints(
    current: RigidTransform,
    target: RigidTransform,
    xy_step_m: float,
    angle_step_degrees: float,
    z_step_m: float,
    transit_z_m: float | None = None,
    final_z_step_m: float | None = None,
) -> list[Waypoint]:
    final_step = z_step_m if final_z_step_m is None else final_z_step_m
    if (
        xy_step_m < 0.0
        or angle_step_degrees <= 0.0
        or z_step_m < 0.0
        or final_step < 0.0
    ):
        raise ValueError(
            "XY/Z waypoint 간격은 0 이상이고 자세 간격은 0보다 커야 합니다"
        )

    result: list[Waypoint] = []
    cx, cy, cz = current.translation
    tx, ty, tz = target.translation
    xy_z = cz if transit_z_m is None else float(transit_z_m)

    if not math.isclose(xy_z, cz, abs_tol=1e-9):
        transit_distance = abs(xy_z - cz)
        transit_count = (
            1
            if z_step_m == 0.0
            else max(1, math.ceil(transit_distance / z_step_m))
        )
        for index in range(1, transit_count + 1):
            ratio = index / transit_count
            result.append(
                Waypoint(
                    "TRANSIT_Z",
                    RigidTransform(
                        (cx, cy, cz + ratio * (xy_z - cz)),
                        current.quaternion,
                    ),
                )
            )

    xy_distance = math.hypot(tx - cx, ty - cy)
    xy_count = (
        1
        if xy_step_m == 0.0
        else max(1, math.ceil(xy_distance / xy_step_m))
    )
    for index in range(1, xy_count + 1):
        ratio = index / xy_count
        result.append(
            Waypoint(
                "XY",
                RigidTransform(
                    (cx + ratio * (tx - cx), cy + ratio * (ty - cy), xy_z),
                    current.quaternion,
                ),
            )
        )

    angle = _quaternion_angle_degrees(current.quaternion, target.quaternion)
    angle_count = max(1, math.ceil(angle / angle_step_degrees))
    for index in range(1, angle_count + 1):
        ratio = index / angle_count
        result.append(
            Waypoint(
                "ROT",
                RigidTransform(
                    (tx, ty, xy_z),
                    _slerp(current.quaternion, target.quaternion, ratio),
                ),
            )
        )

    z_distance = abs(tz - xy_z)
    z_count = (
        1 if final_step == 0.0 else max(1, math.ceil(z_distance / final_step))
    )
    for index in range(1, z_count + 1):
        ratio = index / z_count
        result.append(
            Waypoint(
                "Z",
                RigidTransform(
                    (tx, ty, xy_z + ratio * (tz - xy_z)), target.quaternion
                ),
            )
        )
    return result


def _transform_from_message(message: object) -> RigidTransform:
    translation = message.transform.translation
    rotation = message.transform.rotation
    values = (
        translation.x,
        translation.y,
        translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("TF에 유효하지 않은 숫자가 있습니다")
    return RigidTransform(
        (translation.x, translation.y, translation.z),
        _normalize_quaternion((rotation.x, rotation.y, rotation.z, rotation.w)),
    )


def _save_reference(path: Path, reference: RigidTransform, overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(
            f"기준 파일이 이미 있습니다: {path}\n"
            "다시 저장하려면 teach --overwrite를 사용하세요"
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "version": 1,
        "description": "T_usb_port_joint6_flange_pre",
        "translation_m": list(reference.translation),
        "quaternion_xyzw": list(reference.quaternion),
    }
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")


def _load_reference(path: Path) -> RigidTransform:
    data = json.loads(path.read_text())
    if data.get("version") != 1:
        raise ValueError(f"지원하지 않는 기준 파일 버전입니다: {data.get('version')}")
    translation = tuple(float(value) for value in data["translation_m"])
    quaternion = _normalize_quaternion(data["quaternion_xyzw"])
    if len(translation) != 3 or not all(math.isfinite(value) for value in translation):
        raise ValueError("기준 파일의 translation 형식이 잘못됐습니다")
    return RigidTransform(translation, quaternion)


class UsbPreApproach(Node):
    def __init__(self, base_frame: str, usb_frame: str, flange_frame: str) -> None:
        super().__init__("pinkk_usb_pre_approach")
        self.base_frame = base_frame
        self.usb_frame = usb_frame
        self.flange_frame = flange_frame
        self._joint_lock = threading.Lock()
        self._latest_joints: list[float] | None = None
        self._latest_joint_received_at: float | None = None
        self._active_goal_handle: object | None = None
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0), node=self)
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._ik = self.create_client(GetPositionIK, IK_SERVICE)
        self._action = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def _joint_callback(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if not all(name in values for name in JOINT_NAMES):
            return
        ordered = [float(values[name]) for name in JOINT_NAMES]
        if all(math.isfinite(value) for value in ordered):
            with self._joint_lock:
                self._latest_joints = ordered
                self._latest_joint_received_at = time.monotonic()

    def _wait_future(self, future: object, timeout: float) -> object:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError("ROS 응답 대기 시간이 초과됐습니다")
            time.sleep(0.05)
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()

    def lookup(self, parent: str, child: str) -> RigidTransform:
        try:
            message = self._tf_buffer.lookup_transform(
                parent, child, Time(), Duration(seconds=5.0)
            )
        except TransformException as error:
            raise RuntimeError(f"TF를 읽지 못했습니다: {parent} -> {child}: {error}") from error
        return _transform_from_message(message)

    def wait_for_joints(self) -> list[float]:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            with self._joint_lock:
                if self._latest_joints is not None:
                    return list(self._latest_joints)
            time.sleep(0.05)
        raise RuntimeError("/joint_states를 10초 안에 받지 못했습니다")

    def solve_ik(
        self,
        target: RigidTransform,
        seed: Sequence[float],
        check_collisions: bool,
    ) -> list[float] | None:
        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm_group"
        request.ik_request.ik_link_name = self.flange_frame
        request.ik_request.pose_stamped = self._pose_stamped(target)
        request.ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
        request.ik_request.robot_state.joint_state.position = list(seed)
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = check_collisions
        request.ik_request.timeout = Duration(seconds=2.0).to_msg()
        response = self._wait_future(self._ik.call_async(request), 5.0)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warning(
                f"IK 응답 실패: MoveItErrorCodes={response.error_code.val}"
            )
            return None
        values = dict(
            zip(response.solution.joint_state.name, response.solution.joint_state.position)
        )
        if not all(name in values for name in JOINT_NAMES):
            return None
        return [float(values[name]) for name in JOINT_NAMES]

    def _pose_stamped(self, transform: RigidTransform) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            transform.translation
        )
        x, y, z, w = transform.quaternion
        pose.pose.orientation = Quaternion(x=x, y=y, z=z, w=w)
        return pose

    def move(self, joints: Sequence[float], seconds: float) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(joints)
        point.time_from_start = Duration(seconds=seconds).to_msg()
        goal.trajectory.points = [point]
        handle = self._wait_future(self._action.send_goal_async(goal), 5.0)
        if not handle.accepted:
            raise RuntimeError("trajectory 목표가 거절됐습니다")
        self._active_goal_handle = handle
        try:
            wrapped = self._wait_future(
                handle.get_result_async(), seconds + 15.0
            )
            if wrapped.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(
                    f"trajectory 실행 실패: {wrapped.result.error_code} "
                    f"{wrapped.result.error_string}"
                )
        finally:
            self._active_goal_handle = None

    def cancel_active_move(self) -> None:
        """진행 중인 관절 action goal이 있으면 취소 응답을 기다린다."""
        handle = self._active_goal_handle
        if handle is None:
            return
        self.get_logger().warning("진행 중인 관절 이동 취소 요청")
        try:
            response = self._wait_future(handle.cancel_goal_async(), 3.0)
            if not response.goals_canceling:
                self.get_logger().error("관절 이동 취소가 수락되지 않았습니다")
        except Exception as error:
            self.get_logger().error(f"관절 이동 취소 요청 실패: {error}")
        finally:
            self._active_goal_handle = None

    def verify_joint_hold(
        self,
        target: Sequence[float],
        seconds: float,
        tolerance_degrees: float,
    ) -> None:
        """지정 시간 동안 fresh joint state가 목표 자세를 유지하는지 확인한다."""
        if seconds <= 0.0 or tolerance_degrees <= 0.0:
            raise ValueError("자세 유지 시간과 허용오차는 0보다 커야 합니다")
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            with self._joint_lock:
                actual = (
                    None
                    if self._latest_joints is None
                    else list(self._latest_joints)
                )
                received_at = self._latest_joint_received_at
            if (
                actual is None
                or received_at is None
                or time.monotonic() - received_at > 0.5
            ):
                raise RuntimeError(
                    "관절 자세 유지 검사 중 fresh /joint_states가 없습니다"
                )
            maximum_error = max(
                abs(
                    math.degrees(
                        math.remainder(
                            target_value - actual_value,
                            2.0 * math.pi,
                        )
                    )
                )
                for target_value, actual_value in zip(
                    target, actual, strict=True
                )
            )
            if maximum_error > tolerance_degrees:
                raise RuntimeError(
                    "관측 자세 유지 실패: "
                    f"최대 관절 오차={maximum_error:.3f}deg"
                )
            time.sleep(0.1)
        self.get_logger().info(
            f"관측 자세 {seconds:.1f}초 유지 확인: "
            f"최대 허용오차={tolerance_degrees:.2f}deg"
        )


def _format_transform(name: str, transform: RigidTransform) -> str:
    xyz_mm = [round(value * 1000.0, 2) for value in transform.translation]
    quaternion = [round(value, 6) for value in transform.quaternion]
    return f"{name}: xyz_mm={xyz_mm}, quaternion_xyzw={quaternion}"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="USB TF 기반 flange PRE 단계 접근 (Jupyter 불필요)"
    )
    parser.add_argument("command", choices=("observe", "teach", "show", "run"))
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--base-frame", default="g_base")
    parser.add_argument("--usb-frame", default="usb_port")
    parser.add_argument("--flange-frame", default="joint6_flange")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument(
        "--use-taught-reference",
        action="store_true",
        help="기본 시험값 대신 teach로 저장한 전체 3D 상대 자세 사용",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--standoff-mm",
        type=float,
        default=100.0,
        help="USB Z에서 멈출 flange 높이 오프셋(mm). 기본값 100은 USB보다 10cm 위",
    )
    parser.add_argument("--fixed-roll-deg", type=float, default=-180.0)
    parser.add_argument("--fixed-pitch-deg", type=float, default=0.0)
    parser.add_argument(
        "--yaw-offset-deg",
        type=float,
        default=129.782,
        help=(
            "USB yaw에 더할 flange yaw 고정 보정각(deg). "
            "현재 클릭 축/그리퍼 정렬 기준 기본값 +129.782"
        ),
    )
    parser.add_argument(
        "--check-collisions",
        action="store_true",
        help="IK 계산에서 MoveIt 충돌 검사 사용(기본값: 사용하지 않음)",
    )
    parser.add_argument(
        "--xy-step-mm",
        type=float,
        default=0.0,
        help="XY waypoint 간격(mm). 기본값 0은 중간 분할 없이 한 번에 이동",
    )
    parser.add_argument("--angle-step-deg", type=float, default=10.0)
    parser.add_argument(
        "--z-step-mm",
        type=float,
        default=0.0,
        help="Z waypoint 간격(mm). 기본값 0은 중간 분할 없이 한 번에 이동",
    )
    parser.add_argument(
        "--final-z-step-mm",
        type=float,
        default=10.0,
        help="마지막 USB 접근 Z 간격(mm). 기본값 10, 0이면 한 번에 이동",
    )
    parser.add_argument(
        "--transit-z-mm",
        type=float,
        default=None,
        help="XY 이동 전에 먼저 이동할 g_base 기준 flange Z(mm)",
    )
    parser.add_argument(
        "--motion-seconds",
        type=float,
        default=20.0,
        help="trajectory 계획 시간. 현재 bridge에서는 주로 실행 timeout 계산에 사용",
    )
    parser.add_argument("--settle-seconds", type=float, default=0.2)
    parser.add_argument(
        "--hold-check-seconds",
        type=float,
        default=3.0,
        help="관측 자세 이동 완료 후 유지 검증 시간",
    )
    parser.add_argument(
        "--hold-tolerance-deg",
        type=float,
        default=1.0,
        help="관측 자세 유지 검증의 최대 관절 오차",
    )
    return parser


def _run(node: UsbPreApproach, arguments: argparse.Namespace) -> None:
    if arguments.command == "observe":
        target = [math.radians(value) for value in OBSERVATION_JOINTS_DEG]
        current = node.wait_for_joints()
        node.get_logger().info(
            f"현재 관절각 [deg]: {[round(math.degrees(v), 2) for v in current]}"
        )
        node.get_logger().info(
            f"초기 관측 관절각 [deg]: {list(OBSERVATION_JOINTS_DEG)}"
        )
        if not arguments.execute:
            node.get_logger().warning(
                "DRY RUN입니다. 실제 관측 자세 이동은 명령 끝에 --execute를 붙이세요"
            )
            return
        if arguments.motion_seconds <= 0.0:
            raise ValueError("이동 시간은 0보다 커야 합니다")
        if not node._action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"action server가 없습니다: {ACTION_NAME}")
        node.get_logger().warning("3초 후 초기 관측 자세로 이동합니다")
        time.sleep(3.0)
        node.move(target, arguments.motion_seconds)
        node.verify_joint_hold(
            target,
            arguments.hold_check_seconds,
            arguments.hold_tolerance_deg,
        )
        node.get_logger().info("초기 관측 자세 이동 완료")
        return

    if arguments.command == "teach":
        # lookup_transform(usb, flange)가 곧 T_usb_flange_pre다.
        reference = node.lookup(arguments.usb_frame, arguments.flange_frame)
        _save_reference(arguments.reference, reference, arguments.overwrite)
        node.get_logger().info(_format_transform("저장한 T_usb_flange_pre", reference))
        node.get_logger().info(f"PRE 기준 저장 완료: {arguments.reference}")
        return

    base_usb = node.lookup(arguments.base_frame, arguments.usb_frame)
    current_flange = node.lookup(arguments.base_frame, arguments.flange_frame)
    if arguments.use_taught_reference:
        reference = _load_reference(arguments.reference)
        target_flange = compose(base_usb, reference)
        target_description = "teach로 저장한 T_usb_flange_pre"
    else:
        usb_yaw = _yaw_degrees(base_usb.quaternion)
        target_yaw = usb_yaw + arguments.yaw_offset_deg
        target_flange = RigidTransform(
            (
                base_usb.translation[0],
                base_usb.translation[1],
                base_usb.translation[2] + arguments.standoff_mm / 1000.0,
            ),
            _quaternion_from_rpy_degrees(
                arguments.fixed_roll_deg,
                arguments.fixed_pitch_deg,
                target_yaw,
            ),
        )
        target_description = (
            f"시험값: Z=USB Z+{arguments.standoff_mm:.1f}mm, "
            f"RPY=[{arguments.fixed_roll_deg:.3f}, "
            f"{arguments.fixed_pitch_deg:.3f}, "
            f"USB yaw({usb_yaw:.3f})+{arguments.yaw_offset_deg:.3f}]deg"
        )
    node.get_logger().info(_format_transform("현재 USB", base_usb))
    node.get_logger().info(_format_transform("현재 flange", current_flange))
    node.get_logger().info(f"목표 계산 방식: {target_description}")
    node.get_logger().info(_format_transform("목표 flange PRE", target_flange))
    if (
        not arguments.use_taught_reference
        and target_flange.translation[2] >= current_flange.translation[2]
    ):
        raise ValueError(
            "목표 PRE Z가 현재 flange Z보다 낮지 않아 수직 하강할 수 없습니다: "
            f"current={current_flange.translation[2] * 1000.0:.2f}mm, "
            f"target={target_flange.translation[2] * 1000.0:.2f}mm"
        )
    if arguments.command == "show":
        return

    if arguments.motion_seconds <= 0.0 or arguments.settle_seconds < 0.0:
        raise ValueError("이동 시간은 0보다 크고 정지 대기는 0 이상이어야 합니다")
    waypoints = make_waypoints(
        current_flange,
        target_flange,
        arguments.xy_step_mm / 1000.0,
        arguments.angle_step_deg,
        arguments.z_step_mm / 1000.0,
        None
        if arguments.transit_z_mm is None
        else arguments.transit_z_mm / 1000.0,
        arguments.final_z_step_mm / 1000.0,
    )
    if not node._ik.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(f"IK service가 없습니다: {IK_SERVICE}")
    if arguments.execute and not node._action.wait_for_server(timeout_sec=10.0):
        raise RuntimeError(f"action server가 없습니다: {ACTION_NAME}")

    seed = node.wait_for_joints()
    solutions: list[tuple[Waypoint, list[float]]] = []
    stage_counts: dict[str, int] = {}
    for index, waypoint in enumerate(waypoints, start=1):
        solution = node.solve_ik(
            waypoint.transform, seed, arguments.check_collisions
        )
        if solution is None:
            raise RuntimeError(
                f"IK 사전검사 실패: waypoint={index}/{len(waypoints)}, "
                f"stage={waypoint.stage}, "
                f"xyz_mm={[round(v * 1000.0, 2) for v in waypoint.transform.translation]}"
            )
        seed = solution
        solutions.append((waypoint, solution))
        stage_counts[waypoint.stage] = stage_counts.get(waypoint.stage, 0) + 1

    node.get_logger().info(
        "IK 사전검사 성공: "
        + ", ".join(f"{stage}={count}" for stage, count in stage_counts.items())
    )
    if not arguments.execute:
        node.get_logger().warning(
            "DRY RUN 완료입니다. 실제 이동은 같은 명령 끝에 --execute를 붙이세요"
        )
        return

    node.get_logger().warning("3초 후 XY → 자세 → Z 순서로 실제 이동합니다")
    time.sleep(3.0)
    previous_stage = ""
    for index, (waypoint, solution) in enumerate(solutions, start=1):
        if waypoint.stage != previous_stage:
            node.get_logger().info(f"[{waypoint.stage}] 단계 시작")
            previous_stage = waypoint.stage
        node.move(solution, arguments.motion_seconds)
        if arguments.settle_seconds:
            time.sleep(arguments.settle_seconds)
        node.get_logger().info(
            f"이동 완료 {index}/{len(solutions)} ({waypoint.stage})"
        )
    node.get_logger().info("USB PRE 접근 완료")


def main(args: list[str] | None = None) -> int:
    raw_args = sys.argv if args is None else [sys.argv[0], *args]
    arguments = _build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    rclpy.init(args=args)
    node = UsbPreApproach(
        arguments.base_frame, arguments.usb_frame, arguments.flange_frame
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    exit_code = 0
    try:
        _run(node, arguments)
    except KeyboardInterrupt:
        node.cancel_active_move()
        node.get_logger().warning("사용자가 중단했습니다")
        exit_code = 130
    except Exception as error:
        node.get_logger().error(f"USB PRE 접근 실패: {error}")
        exit_code = 1
    finally:
        node.cancel_active_move()
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
