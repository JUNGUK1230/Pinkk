"""URDF 직렬 체인과 damped Jacobian 계산 시험."""

from pathlib import Path

import numpy as np
import pytest

from pinkk_usb_insertion.control.serial_chain import (
    SerialChainModel,
    damped_joint_step,
    rotation_vector_error,
)


URDF = """
<robot name="test">
  <link name="base"/><link name="link1"/><link name="tip"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="tip_fixed" type="fixed">
    <parent link="link1"/><child link="tip"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"""


def test_serial_chain_geometric_jacobian(tmp_path: Path) -> None:
    """단일 Z 회전관절의 선속도/각속도 Jacobian을 확인한다."""
    path = tmp_path / 'test.urdf'
    path.write_text(URDF, encoding='utf-8')
    model = SerialChainModel.from_urdf_file(path, 'base', 'tip')

    transform, jacobian = model.forward_and_jacobian({'joint1': 0.0})

    assert model.active_joints == ('joint1',)
    assert np.allclose(transform[:3, 3], (1.0, 0.0, 0.0))
    assert np.allclose(jacobian[:, 0], (0.0, 1.0, 0.0, 0.0, 0.0, 1.0))


def test_rotation_vector_error_about_z() -> None:
    """Z축 90도 회전 오차가 회전벡터로 변환되는지 확인한다."""
    target = np.asarray(((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))
    error = rotation_vector_error(target, np.eye(3))
    assert np.allclose(error, (0.0, 0.0, np.pi / 2.0))


def test_damped_joint_step_clamps_largest_axis() -> None:
    """DLS 관절 증분의 축별 최대값이 제한되는지 확인한다."""
    jacobian = np.eye(6)
    delta = damped_joint_step(
        jacobian,
        np.ones(6),
        damping=0.01,
        orientation_scale_m_per_rad=0.05,
        maximum_joint_step_rad=0.1,
    )
    assert np.max(np.abs(delta)) == pytest.approx(0.1)
