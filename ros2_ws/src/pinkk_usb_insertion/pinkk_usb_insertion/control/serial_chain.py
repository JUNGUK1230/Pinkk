"""작은 관절 증분 시험을 위한 URDF 직렬 체인 전진기구학/Jacobian."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def _values(text: str | None, count: int, default: tuple[float, ...]):
    if text is None:
        return np.asarray(default, dtype=np.float64)
    values = np.asarray(
        [float(value) for value in text.split()], dtype=np.float64
    )
    if values.shape != (count,) or not np.all(np.isfinite(values)):
        raise ValueError(f'URDF 값은 유한한 숫자 {count}개여야 합니다: {text}')
    return values


def _rpy_rotation(rpy) -> np.ndarray:
    roll, pitch, yaw = np.asarray(rpy, dtype=np.float64)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=np.float64,
    )


def _transform(xyz, rpy) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = _rpy_rotation(rpy)
    result[:3, 3] = np.asarray(xyz, dtype=np.float64)
    return result


def _axis_rotation(axis, angle: float) -> np.ndarray:
    unit = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(unit))
    if norm < 1e-12:
        raise ValueError('회전관절 axis 길이가 0입니다')
    x, y, z = unit / norm
    c, s = math.cos(angle), math.sin(angle)
    cross = 1.0 - c
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(
        (
            (c + x * x * cross, x * y * cross - z * s, x * z * cross + y * s),
            (y * x * cross + z * s, c + y * y * cross, y * z * cross - x * s),
            (z * x * cross - y * s, z * y * cross + x * s, c + z * z * cross),
        ),
        dtype=np.float64,
    )
    return result


@dataclass(frozen=True)
class ChainJoint:
    """URDF 체인의 관절 하나를 표현한다."""

    name: str
    joint_type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float | None
    upper: float | None


class SerialChainModel:
    """URDF의 base→tip 단일 체인에 대한 geometric Jacobian 모델."""

    def __init__(self, joints: list[ChainJoint], base: str, tip: str):
        """파싱된 base→tip 관절 목록을 저장한다."""
        self.joints = list(joints)
        self.base = base
        self.tip = tip
        self.active_joints = tuple(
            joint.name
            for joint in joints
            if joint.joint_type in ('revolute', 'continuous')
        )

    @classmethod
    def from_urdf_file(cls, path: str | Path, base: str, tip: str):
        """URDF 파일에서 base→tip 직렬 체인을 만든다."""
        root = ET.parse(Path(path)).getroot()
        by_child: dict[str, ChainJoint] = {}
        for element in root.findall('joint'):
            joint_type = element.attrib.get('type', '')
            parent = element.find('parent').attrib['link']
            child = element.find('child').attrib['link']
            origin_element = element.find('origin')
            xyz = _values(
                None
                if origin_element is None
                else origin_element.attrib.get('xyz'),
                3,
                (0.0, 0.0, 0.0),
            )
            rpy = _values(
                None
                if origin_element is None
                else origin_element.attrib.get('rpy'),
                3,
                (0.0, 0.0, 0.0),
            )
            axis_element = element.find('axis')
            axis = _values(
                None
                if axis_element is None
                else axis_element.attrib.get('xyz'),
                3,
                (1.0, 0.0, 0.0),
            )
            limit = element.find('limit')
            lower = (
                None
                if limit is None or 'lower' not in limit.attrib
                else float(limit.attrib['lower'])
            )
            upper = (
                None
                if limit is None or 'upper' not in limit.attrib
                else float(limit.attrib['upper'])
            )
            by_child[child] = ChainJoint(
                name=element.attrib['name'],
                joint_type=joint_type,
                parent=parent,
                child=child,
                origin=_transform(xyz, rpy),
                axis=axis,
                lower=lower,
                upper=upper,
            )
        reversed_chain: list[ChainJoint] = []
        link = tip
        while link != base:
            if link not in by_child:
                raise ValueError(
                    f'URDF에서 {base}→{tip} 체인을 찾지 못했습니다: {link}'
                )
            joint = by_child[link]
            reversed_chain.append(joint)
            link = joint.parent
        return cls(list(reversed(reversed_chain)), base, tip)

    def forward_and_jacobian(self, positions: dict[str, float]):
        """주어진 관절값의 tip transform과 geometric Jacobian을 계산한다."""
        transform = np.eye(4, dtype=np.float64)
        active: list[tuple[np.ndarray, np.ndarray]] = []
        for joint in self.joints:
            transform = transform @ joint.origin
            if joint.joint_type in ('revolute', 'continuous'):
                if joint.name not in positions:
                    raise ValueError(f'관절 위치가 없습니다: {joint.name}')
                axis_world = transform[:3, :3] @ joint.axis
                axis_world = axis_world / np.linalg.norm(axis_world)
                active.append((transform[:3, 3].copy(), axis_world.copy()))
                transform = transform @ _axis_rotation(
                    joint.axis, float(positions[joint.name])
                )
            elif joint.joint_type != 'fixed':
                raise ValueError(f'지원하지 않는 관절 형식: {joint.joint_type}')
        end_position = transform[:3, 3]
        jacobian = np.zeros((6, len(active)), dtype=np.float64)
        for index, (joint_position, axis_world) in enumerate(active):
            jacobian[:3, index] = np.cross(
                axis_world, end_position - joint_position
            )
            jacobian[3:, index] = axis_world
        return transform, jacobian

    def clamp_to_limits(self, names, positions, margin_rad: float):
        """관절 목표를 URDF 한계와 지정 여유 안으로 제한한다."""
        limits = {joint.name: joint for joint in self.joints}
        result = np.asarray(positions, dtype=np.float64).copy()
        for index, name in enumerate(names):
            joint = limits[name]
            if joint.lower is not None:
                result[index] = max(result[index], joint.lower + margin_rad)
            if joint.upper is not None:
                result[index] = min(result[index], joint.upper - margin_rad)
        return result


def rotation_vector_error(target_rotation, actual_rotation) -> np.ndarray:
    """Base 좌표계에서 actual→target 최단 회전벡터[rad]를 반환한다."""
    relative = np.asarray(target_rotation) @ np.asarray(actual_rotation).T
    vector = np.asarray(
        (
            relative[2, 1] - relative[1, 2],
            relative[0, 2] - relative[2, 0],
            relative[1, 0] - relative[0, 1],
        ),
        dtype=np.float64,
    )
    cosine = max(-1.0, min(1.0, (float(np.trace(relative)) - 1.0) * 0.5))
    angle = math.acos(cosine)
    if angle < 1e-9:
        return 0.5 * vector
    sine = math.sin(angle)
    if abs(sine) < 1e-8:
        raise ValueError(
            '180도 부근 회전 오차는 Jacobian 시험에서 지원하지 않습니다'
        )
    return vector * (angle / (2.0 * sine))


def damped_joint_step(
    jacobian,
    task_error,
    damping: float,
    orientation_scale_m_per_rad: float,
    maximum_joint_step_rad: float,
) -> np.ndarray:
    """가중 damped least-squares 관절 증분을 계산하고 축별 상한을 적용한다."""
    matrix = np.asarray(jacobian, dtype=np.float64)
    error = np.asarray(task_error, dtype=np.float64).reshape(6)
    if matrix.shape[0] != 6 or matrix.shape[1] < 1:
        raise ValueError('Jacobian 크기가 잘못됐습니다')
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(error)):
        raise ValueError('Jacobian/error는 유한값이어야 합니다')
    if damping <= 0.0 or orientation_scale_m_per_rad <= 0.0:
        raise ValueError('damping과 자세 scale은 양수여야 합니다')
    if maximum_joint_step_rad <= 0.0:
        raise ValueError('최대 관절 step은 양수여야 합니다')
    weights = np.diag((1.0, 1.0, 1.0) + (orientation_scale_m_per_rad,) * 3)
    weighted_jacobian = weights @ matrix
    weighted_error = weights @ error
    regularized = (
        weighted_jacobian @ weighted_jacobian.T
        + (damping * damping) * np.eye(6)
    )
    delta = weighted_jacobian.T @ np.linalg.solve(regularized, weighted_error)
    maximum = float(np.max(np.abs(delta)))
    if maximum > maximum_joint_step_rad:
        delta *= maximum_joint_step_rad / maximum
    return delta
