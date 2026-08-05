"""상단 카메라 localization의 최신 차량 중심 pose를 ROS 2로 발행한다."""

import argparse
import json
import math
from pathlib import Path
import time
from typing import Any, Sequence


REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_SCENE_PATH = (
    REPO_ROOT / "src/central_control/path_planning/output/live_vision_scene.json"
)
POSE_TOPIC = "/pinkk/vehicle_pose"


def load_current_vehicle_pose(
    scene_path: str | Path,
    max_age_sec: float,
    now_unix_sec: float | None = None,
) -> tuple[float, float, float, int]:
    """planning-ready인 최신 차량 중심 pose를 m 단위로 반환한다.

    카메라 차량 검출이 없거나 heading/ego 선택이 모호한 프레임은 제어기에
    전달하지 않는다. 오래된 좌표도 현재 차량 위치로 오인되지 않게 거부한다.
    """
    if max_age_sec <= 0.0:
        raise ValueError("max_age_sec must be positive")
    source = Path(scene_path)
    if not source.exists():
        raise FileNotFoundError(f"live vision scene not found: {source}")
    with source.open(encoding="utf-8") as file:
        scene: dict[str, Any] = json.load(file)
    if not scene.get("planning_ready", False):
        raise ValueError(f"scene is not planning-ready: {scene.get('status')}")
    observed_at = scene.get("observed_at_unix_sec")
    if not isinstance(observed_at, (int, float)):
        raise ValueError("scene has no observed_at_unix_sec")
    now = time.time() if now_unix_sec is None else now_unix_sec
    age = now - float(observed_at)
    if age < -1.0 or age > max_age_sec:
        raise ValueError(f"scene is stale ({age:.3f} sec)")
    vehicle = scene.get("vehicle")
    if not isinstance(vehicle, dict):
        raise ValueError("scene has no vehicle")
    vehicle_center = vehicle.get("center_cm")
    yaw_rad = vehicle.get("yaw_rad")
    if (
        not isinstance(vehicle_center, list)
        or len(vehicle_center) != 2
        or not isinstance(yaw_rad, (int, float))
    ):
        raise ValueError("scene vehicle pose is invalid")
    frame_index = scene.get("frame_index")
    if not isinstance(frame_index, int):
        raise ValueError("scene frame_index is invalid")
    return (
        float(vehicle_center[0]) / 100.0,
        float(vehicle_center[1]) / 100.0,
        float(yaw_rad),
        frame_index,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish fresh camera-localized vehicle pose to ROS 2."
    )
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE_PATH)
    parser.add_argument("--topic", default=POSE_TOPIC)
    parser.add_argument("--max-age-sec", type=float, default=0.5)
    parser.add_argument("--poll-period-sec", type=float, default=0.05)
    return parser.parse_args()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args() if argv is None else _parse_args_from(argv)
    if args.poll_period_sec <= 0.0:
        raise ValueError("poll-period-sec must be positive")
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 environment is required. Source ROS setup.bash before running."
        ) from error

    class VehiclePosePublisher(Node):
        def __init__(self) -> None:
            super().__init__("pinkk_vehicle_pose_publisher")
            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.publisher = self.create_publisher(PoseStamped, args.topic, qos)
            self.last_frame_index: int | None = None
            self.last_warning: str | None = None
            self.timer = self.create_timer(args.poll_period_sec, self.publish_if_fresh)

        def publish_if_fresh(self) -> None:
            try:
                x_m, y_m, yaw_rad, frame_index = load_current_vehicle_pose(
                    args.scene,
                    args.max_age_sec,
                )
                if frame_index == self.last_frame_index:
                    return
                message = PoseStamped()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = "lidar_map"
                message.pose.position.x = x_m
                message.pose.position.y = y_m
                message.pose.position.z = 0.0
                message.pose.orientation.z = math.sin(yaw_rad / 2.0)
                message.pose.orientation.w = math.cos(yaw_rad / 2.0)
                self.publisher.publish(message)
                self.last_frame_index = frame_index
                self.last_warning = None
                self.get_logger().info(
                    f"Published frame={frame_index}: x={x_m:.3f}m, "
                    f"y={y_m:.3f}m, yaw={math.degrees(yaw_rad):.1f}deg"
                )
            except (FileNotFoundError, ValueError, json.JSONDecodeError) as error:
                # 매 timer마다 같은 stale 경고를 반복하지 않게 한 번만 기록한다.
                message = str(error)
                if message != self.last_warning:
                    self.get_logger().warning(f"vehicle pose not published: {message}")
                    self.last_warning = message

    rclpy.init(args=None)
    node = VehiclePosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def _parse_args_from(argv: Sequence[str]) -> argparse.Namespace:
    previous = __import__("sys").argv
    try:
        __import__("sys").argv = ["vehicle_pose_publisher", *argv]
        return parse_args()
    finally:
        __import__("sys").argv = previous


if __name__ == "__main__":
    raise SystemExit(main())
