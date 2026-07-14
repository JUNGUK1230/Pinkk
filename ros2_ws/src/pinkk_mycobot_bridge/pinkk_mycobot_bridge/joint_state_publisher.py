"""Publish measured MyCobot280 joint angles without sending motion commands."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Sequence

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


JOINT_NAMES = (
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
)


def angles_deg_to_rad(values: Sequence[float]) -> list[float]:
    """Validate six measured joint angles and convert degrees to radians."""
    if not isinstance(values, (list, tuple)) or len(values) != len(JOINT_NAMES):
        raise ValueError(f"expected 6 joint angles, got {values!r}")
    numeric = [float(value) for value in values]
    if not all(math.isfinite(value) for value in numeric):
        raise ValueError(f"joint angles contain NaN or inf: {values!r}")
    if any(abs(value) > 360.0 for value in numeric):
        raise ValueError(f"joint angle outside sanity range: {values!r}")
    return [math.radians(value) for value in numeric]


class MyCobotJointStatePublisher(Node):
    """Read the robot encoders and publish ``/joint_states`` at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("pinkk_mycobot_joint_state_publisher")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("frame_id", "base_link")

        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        if rate <= 0.0 or rate > 50.0:
            raise ValueError("publish_rate_hz must be in (0, 50]")
        if not Path(port).exists():
            raise FileNotFoundError(f"robot serial port does not exist: {port}")

        try:
            from pymycobot import MyCobot280
        except ImportError as error:
            raise RuntimeError(
                "pymycobot is not available; activate ~/venv/mycobot before running"
            ) from error

        # This node intentionally calls only get_angles(). It contains no movement API call.
        self._robot = MyCobot280(port, baud)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(JointState, "/joint_states", qos)
        self._consecutive_failures = 0
        self._timer = self.create_timer(1.0 / rate, self._publish_measured_state)
        self.get_logger().info(
            f"READ-ONLY robot connection: port={port}, baud={baud}, rate={rate:.1f}Hz"
        )

    def _publish_measured_state(self) -> None:
        try:
            positions = angles_deg_to_rad(self._robot.get_angles())
        except Exception as error:  # Serial failures vary across pymycobot versions.
            self._consecutive_failures += 1
            if self._consecutive_failures == 1 or self._consecutive_failures % 20 == 0:
                self.get_logger().warning(
                    f"get_angles failed ({self._consecutive_failures} consecutive): {error}"
                )
            return

        self._consecutive_failures = 0
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.name = list(JOINT_NAMES)
        message.position = positions
        self._publisher.publish(message)

    def close(self) -> None:
        close = getattr(self._robot, "close", None)
        if callable(close):
            close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: MyCobotJointStatePublisher | None = None
    try:
        node = MyCobotJointStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
