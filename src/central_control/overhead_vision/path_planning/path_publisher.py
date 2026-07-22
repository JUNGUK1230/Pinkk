"""검증된 Hybrid A* trajectory 파일을 ROS 2 토픽으로 발행한다.

`/pinkk/planned_path`는 Pure Pursuit 같은 표준 path follower용 Pose 경로이고,
`/pinkk/planned_trajectory`는 direction·속도·조향까지 포함한 제어용 행렬이다.
이 노드는 명령을 직접 실행하지 않고, 검증을 통과해 저장된 경로만 발행한다.
"""

import argparse
import json
import math
from pathlib import Path
from typing import Any, Sequence


REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_TRAJECTORY_PATH = (
    REPO_ROOT
    / "src/central_control/path_planning/output/live_hybrid_path_world_cm.json"
)
PATH_TOPIC = "/pinkk/planned_path"
TRAJECTORY_TOPIC = "/pinkk/planned_trajectory"
TRAJECTORY_FIELDS = (
    "x_m",
    "y_m",
    "yaw_rad",
    "direction",
    "target_speed_mps",
    "steer_rad",
    "stop_required",
)


def load_validated_trajectory(path: str | Path) -> dict[str, Any]:
    """파일의 경로 구조와 validator 통과 여부를 확인해 fail-closed 한다."""
    source = Path(path)
    if not source.exists():
        raise FileNotFoundError(f"trajectory file not found: {source}")
    with source.open(encoding="utf-8") as file:
        payload = json.load(file)
    if payload.get("planner") != "hybrid_astar":
        raise ValueError("only validated hybrid_astar trajectory can be published")
    if payload.get("control_ready") is False:
        raise ValueError("trajectory is explicitly marked control_ready=false")
    if not isinstance(payload.get("validation_metrics"), dict):
        raise ValueError("trajectory has no validation_metrics")
    path_rows = payload.get("path")
    if not isinstance(path_rows, list) or not path_rows:
        raise ValueError("trajectory path is empty")
    for index, row in enumerate(path_rows):
        if not isinstance(row, dict):
            raise ValueError(f"trajectory row {index} is not an object")
        required = (
            "x_cm",
            "y_cm",
            "yaw_rad",
            "direction",
            "target_speed_mps",
            "steer_deg",
            "stop_required",
        )
        if any(key not in row for key in required):
            raise ValueError(f"trajectory row {index} misses required fields")
        if int(row["direction"]) not in (-1, 1):
            raise ValueError(f"trajectory row {index} has invalid direction")
    return payload


def trajectory_matrix(payload: dict[str, Any]) -> list[float]:
    """CSV/JSON의 cm 경로를 ROS trajectory 행렬(m 단위)로 평탄화한다."""
    matrix: list[float] = []
    for row in payload["path"]:
        matrix.extend(
            (
                float(row["x_cm"]) / 100.0,
                float(row["y_cm"]) / 100.0,
                float(row["yaw_rad"]),
                float(row["direction"]),
                float(row["target_speed_mps"]),
                math.radians(float(row["steer_deg"])),
                float(row["stop_required"]),
            )
        )
    return matrix


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish validated PINKK Hybrid A* trajectory to ROS 2 topics."
    )
    parser.add_argument("--trajectory", type=Path, default=DEFAULT_TRAJECTORY_PATH)
    parser.add_argument("--path-topic", default=PATH_TOPIC)
    parser.add_argument("--trajectory-topic", default=TRAJECTORY_TOPIC)
    parser.add_argument(
        "--watch-period-sec",
        type=float,
        default=0.5,
        help="Reload and republish when the trajectory JSON modification time changes.",
    )
    return parser.parse_args()


def _yaw_to_quaternion(yaw_rad: float) -> tuple[float, float]:
    """Planar yaw의 quaternion z,w 성분을 반환한다."""
    return math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def main(argv: Sequence[str] | None = None) -> int:
    """ROS 2 publisher를 시작하고 trajectory JSON 변경을 감시한다."""
    args = parse_args() if argv is None else _parse_args_from(argv)
    if args.watch_period_sec <= 0.0:
        raise ValueError("watch-period-sec must be positive")

    # ROS 2가 설치되지 않은 일반 Python 환경에서도 JSON helper test가 가능하도록
    # rclpy import는 실제 publisher 실행 시점까지 늦춘다.
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as RosPath
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Float64MultiArray, MultiArrayDimension
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 environment is required. Source ROS setup.bash before running."
        ) from error

    class TrajectoryPublisher(Node):
        def __init__(self) -> None:
            super().__init__("pinkk_trajectory_publisher")
            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.path_publisher = self.create_publisher(RosPath, args.path_topic, qos)
            self.trajectory_publisher = self.create_publisher(
                Float64MultiArray,
                args.trajectory_topic,
                qos,
            )
            self.last_mtime_ns: int | None = None
            self.timer = self.create_timer(args.watch_period_sec, self.publish_if_updated)
            self.publish_if_updated(force=True)

        def publish_if_updated(self, force: bool = False) -> None:
            try:
                mtime_ns = args.trajectory.stat().st_mtime_ns
                if not force and mtime_ns == self.last_mtime_ns:
                    return
                payload = load_validated_trajectory(args.trajectory)
                self._publish(payload)
                self.last_mtime_ns = mtime_ns
            except (FileNotFoundError, ValueError, json.JSONDecodeError) as error:
                # 경로 생성 중 atomic replace 직전이거나 planner가 실패한 경우에는
                # 마지막 정상 경로를 덮어쓰지 않고 이유만 경고한다.
                self.get_logger().warning(f"trajectory not published: {error}")

        def _publish(self, payload: dict[str, Any]) -> None:
            stamp = self.get_clock().now().to_msg()
            path_message = RosPath()
            path_message.header.stamp = stamp
            path_message.header.frame_id = "lidar_map"
            for row in payload["path"]:
                pose = PoseStamped()
                pose.header = path_message.header
                pose.pose.position.x = float(row["x_cm"]) / 100.0
                pose.pose.position.y = float(row["y_cm"]) / 100.0
                pose.pose.position.z = 0.0
                pose.pose.orientation.z, pose.pose.orientation.w = _yaw_to_quaternion(
                    float(row["yaw_rad"])
                )
                path_message.poses.append(pose)

            trajectory_message = Float64MultiArray()
            points = len(payload["path"])
            point_dim = MultiArrayDimension()
            point_dim.label = "point"
            point_dim.size = points
            point_dim.stride = points * len(TRAJECTORY_FIELDS)
            field_dim = MultiArrayDimension()
            field_dim.label = "fields=" + ",".join(TRAJECTORY_FIELDS)
            field_dim.size = len(TRAJECTORY_FIELDS)
            field_dim.stride = len(TRAJECTORY_FIELDS)
            trajectory_message.layout.dim = [point_dim, field_dim]
            trajectory_message.data = trajectory_matrix(payload)

            self.path_publisher.publish(path_message)
            self.trajectory_publisher.publish(trajectory_message)
            self.get_logger().info(
                f"Published {points} points: {args.path_topic}, {args.trajectory_topic}"
            )

    rclpy.init(args=None)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def _parse_args_from(argv: Sequence[str]) -> argparse.Namespace:
    """단위 테스트용 argv parser."""
    previous = __import__("sys").argv
    try:
        __import__("sys").argv = ["path_publisher", *argv]
        return parse_args()
    finally:
        __import__("sys").argv = previous


if __name__ == "__main__":
    raise SystemExit(main())
