"""ROS pose와 MyCobot Cartesian 좌표 사이의 단위·회전 변환."""

from __future__ import annotations

from collections.abc import Sequence
import math


def _finite(values: Sequence[float], expected: int, label: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != expected or not all(math.isfinite(value) for value in result):
        raise ValueError(f'{label}은 유한한 값 {expected}개여야 합니다')
    return result


def normalize_quaternion(values: Sequence[float]) -> tuple[float, float, float, float]:
    """XY/ZW quaternion을 정규화한다."""
    x, y, z, w = _finite(values, 4, 'quaternion')
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('길이가 0인 quaternion입니다')
    return x / norm, y / norm, z / norm, w / norm


def quaternion_to_rpy_degrees(values: Sequence[float]) -> tuple[float, float, float]:
    """
    XY/ZW quaternion을 MyCobot [rx, ry, rz] degree로 바꾼다.

    Hand-Eye에서 검증한 intrinsic ZYX, 즉 R=Rz(yaw)Ry(pitch)Rx(roll)을 쓴다.
    """
    x, y, z, w = normalize_quaternion(values)
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return tuple(
        (math.degrees(value) + 180.0) % 360.0 - 180.0
        for value in (roll, pitch, yaw)
    )


def rpy_degrees_to_quaternion(
    roll_deg: float, pitch_deg: float, yaw_deg: float
) -> tuple[float, float, float, float]:
    """로봇의 [rx, ry, rz] degree를 XY/ZW quaternion으로 바꾼다."""
    roll, pitch, yaw = (
        math.radians(value) * 0.5
        for value in _finite((roll_deg, pitch_deg, yaw_deg), 3, 'RPY')
    )
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return normalize_quaternion(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def pose_values_to_robot_coords(
    position_m: Sequence[float], quaternion_xyzw: Sequence[float]
) -> list[float]:
    """ROS meter/xyzw pose를 MyCobot mm/[rx,ry,rz] 좌표로 바꾼다."""
    position = _finite(position_m, 3, 'position')
    roll, pitch, yaw = quaternion_to_rpy_degrees(quaternion_xyzw)
    return [*(value * 1000.0 for value in position), roll, pitch, yaw]


def robot_coords_to_pose_values(
    coords: Sequence[float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """로봇의 mm/degree 좌표를 ROS meter/XY/ZW pose 값으로 바꾼다."""
    x, y, z, roll, pitch, yaw = _finite(coords, 6, 'robot coords')
    return (
        (x / 1000.0, y / 1000.0, z / 1000.0),
        rpy_degrees_to_quaternion(roll, pitch, yaw),
    )


def wrapped_angle_difference_deg(first_deg: float, second_deg: float) -> float:
    """두 degree 각도의 -180~180 범위 최단 차이를 반환한다."""
    first, second = _finite((first_deg, second_deg), 2, 'angles')
    return (first - second + 180.0) % 360.0 - 180.0


def pose_error(
    target_position: Sequence[float],
    target_quaternion: Sequence[float],
    actual_position: Sequence[float],
    actual_quaternion: Sequence[float],
) -> tuple[float, float]:
    """두 pose 사이의 위치[m]와 최단 회전각[deg] 오차를 반환한다."""
    target = _finite(target_position, 3, 'target position')
    actual = _finite(actual_position, 3, 'actual position')
    position_error = math.sqrt(sum((a - b) ** 2 for a, b in zip(target, actual)))
    first = normalize_quaternion(target_quaternion)
    second = normalize_quaternion(actual_quaternion)
    dot = min(1.0, abs(sum(a * b for a, b in zip(first, second))))
    orientation_error = math.degrees(2.0 * math.acos(dot))
    return position_error, orientation_error
