"""ROS 메시지와 내부 4×4 transform 사이의 변환 도우미."""

from __future__ import annotations

from geometry_msgs.msg import Pose
import numpy as np

from .geometry.transforms import make_transform, rotation_to_quaternion, validate_transform


def transform_to_pose(transform: np.ndarray) -> Pose:
    matrix = validate_transform(transform)
    quaternion = rotation_to_quaternion(matrix[:3, :3])
    message = Pose()
    message.position.x, message.position.y, message.position.z = matrix[:3, 3]
    message.orientation.x, message.orientation.y = quaternion[:2]
    message.orientation.z, message.orientation.w = quaternion[2:]
    return message


def pose_to_transform(message: Pose) -> np.ndarray:
    return make_transform(
        (message.position.x, message.position.y, message.position.z),
        (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ),
    )
