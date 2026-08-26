"""do-mpc/Matplotlib 기반 실시간 MPC 상태·예측·입력 시각화 노드."""

from __future__ import annotations

import json
import math
import threading
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="The ONNX feature is not available.*")
warnings.filterwarnings("ignore", message="The opcua feature is not available.*")

try:
    import do_mpc
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    import rclpy
    from nav_msgs.msg import Path
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
except ImportError as error:  # pragma: no cover - 실행 환경 안내용
    raise RuntimeError(
        "ROS 2 환경을 source하고 requirements.txt를 설치한 뒤 실행해야 합니다"
    ) from error


class MpcVisualizationNode(Node):
    """MPC debug snapshot과 전체 path를 GUI thread에 안전하게 전달한다."""

    def __init__(self) -> None:
        super().__init__("pinkk_mpc_visualizer")
        self.declare_parameter("debug_topic", "mpc_debug")
        self.declare_parameter("path_topic", "path")
        self.declare_parameter("refresh_hz", 10.0)
        self.declare_parameter("history_sec", 30.0)
        self.refresh_hz = max(1.0, float(self.get_parameter("refresh_hz").value))
        self.history_sec = max(5.0, float(self.get_parameter("history_sec").value))

        debug_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("debug_topic").value),
            self._debug_callback,
            debug_qos,
        )
        self.create_subscription(
            Path,
            str(self.get_parameter("path_topic").value),
            self._path_callback,
            path_qos,
        )
        self._lock = threading.Lock()
        self._snapshot: dict | None = None
        self._snapshot_sequence = 0
        self._path_xy = np.empty((0, 2), dtype=np.float64)

    def _debug_callback(self, message: String) -> None:
        try:
            snapshot = json.loads(message.data)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(f"Invalid MPC debug JSON: {error}")
            return
        with self._lock:
            self._snapshot = snapshot
            self._snapshot_sequence += 1

    def _path_callback(self, message: Path) -> None:
        path_xy = np.asarray(
            [
                (float(pose.pose.position.x), float(pose.pose.position.y))
                for pose in message.poses
            ],
            dtype=np.float64,
        )
        with self._lock:
            self._path_xy = path_xy

    def latest(self) -> tuple[dict | None, int, np.ndarray]:
        with self._lock:
            snapshot = None if self._snapshot is None else dict(self._snapshot)
            return snapshot, self._snapshot_sequence, self._path_xy.copy()


def _build_do_mpc_data():
    """외부 MPC snapshot을 담을 do-mpc model/data 컨테이너를 만든다."""
    model = do_mpc.model.Model("discrete")
    for name in ("x", "y", "yaw", "cross_track", "heading_error"):
        model.set_variable("_x", name)
    for name in ("speed", "angular_speed", "curvature"):
        model.set_variable("_u", name)
    for name in (
        "speed_utilization",
        "curvature_utilization",
        "angular_utilization",
        "solve_utilization",
        "cost",
        "progress",
    ):
        model.set_variable("_tvp", name)
    for name in ("x", "y", "yaw", "cross_track", "heading_error"):
        model.set_rhs(name, model.x[name])
    model.setup()
    return do_mpc.data.Data(model)


class MpcDashboard:
    def __init__(self, node: MpcVisualizationNode) -> None:
        self.node = node
        self.data = _build_do_mpc_data()
        self.graphics = do_mpc.graphics.Graphics(self.data)
        self.first_stamp: float | None = None
        self.last_sequence = -1

        self.figure, axes = plt.subplots(2, 2, figsize=(14, 8))
        self.path_axis, self.input_axis = axes[0]
        self.error_axis, self.condition_axis = axes[1]
        self.figure.canvas.manager.set_window_title("PINKK do-mpc Live Dashboard")

        self.path_line, = self.path_axis.plot([], [], color="0.65", linewidth=2, label="full path")
        self.actual_line, = self.path_axis.plot([], [], "b-", linewidth=2, label="actual history")
        self.prediction_line, = self.path_axis.plot([], [], "m-o", markersize=4, linewidth=2, label="MPC prediction")
        self.reference_line, = self.path_axis.plot([], [], "g--o", markersize=3, linewidth=1.5, label="reference horizon")
        self.vehicle_point, = self.path_axis.plot([], [], "ro", markersize=8, label="current pose")
        self.heading_line, = self.path_axis.plot([], [], "r-", linewidth=3)
        self.path_axis.set_title("Current state and optimized future trajectory")
        self.path_axis.set_xlabel("lidar_map x [m]")
        self.path_axis.set_ylabel("lidar_map y [m]")
        self.path_axis.set_aspect("equal", adjustable="datalim")
        self.path_axis.grid(True)
        self.path_axis.legend(loc="best", fontsize=8)

        self.graphics.add_line("_u", "speed", self.input_axis, label="v [m/s]")
        self.graphics.add_line("_u", "angular_speed", self.input_axis, label="omega [rad/s]")
        self.graphics.add_line("_u", "curvature", self.input_axis, label="curvature [1/m]")
        self.input_axis.set_title("Applied control input")
        self.input_axis.set_xlabel("time [s]")
        self.input_axis.grid(True)
        self.input_axis.legend(loc="upper left", fontsize=8)

        self.graphics.add_line("_x", "cross_track", self.error_axis, label="cross-track [m]")
        self.graphics.add_line("_x", "heading_error", self.error_axis, label="heading error [rad]")
        self.error_axis.axhline(0.0, color="0.4", linewidth=1)
        self.error_axis.set_title("Tracking error")
        self.error_axis.set_xlabel("time [s]")
        self.error_axis.grid(True)
        self.error_axis.legend(loc="upper left", fontsize=8)

        for name, label in (
            ("speed_utilization", "|v| / limit"),
            ("curvature_utilization", "|curvature| / limit"),
            ("angular_utilization", "|omega| / limit"),
            ("solve_utilization", "solve time / dt"),
        ):
            self.graphics.add_line("_tvp", name, self.condition_axis, label=label)
        self.condition_axis.axhline(1.0, color="r", linestyle="--", linewidth=1, label="constraint")
        self.condition_axis.set_title("Control-condition utilization")
        self.condition_axis.set_xlabel("time [s]")
        self.condition_axis.set_ylabel("ratio")
        self.condition_axis.grid(True)
        self.condition_axis.legend(loc="upper left", fontsize=8)

        self.status_text = self.figure.text(0.5, 0.985, "WAITING_FOR_MPC_DEBUG", ha="center", va="top")
        self.actual_xy: list[tuple[float, float]] = []
        self.figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))

    @staticmethod
    def _safe_ratio(value: float, limit: float) -> float:
        return abs(value) / limit if limit > 1e-9 else 0.0

    def update(self, _frame: int):
        snapshot, sequence, path_xy = self.node.latest()
        if path_xy.size:
            self.path_line.set_data(path_xy[:, 0], path_xy[:, 1])
        if snapshot is None or sequence == self.last_sequence:
            return ()
        self.last_sequence = sequence

        state = snapshot.get("state")
        command = snapshot.get("command", [0.0, 0.0, 0.0])
        errors = snapshot.get("errors", [0.0, 0.0])
        solver = snapshot.get("solver", [0.0, 0.0])
        limits = snapshot.get("limits", [0.0, 0.0, 0.0])
        stamp = float(snapshot.get("stamp_sec", 0.0))
        if self.first_stamp is None:
            self.first_stamp = stamp
        elapsed = max(0.0, stamp - self.first_stamp)

        if state is not None:
            self.actual_xy.append((float(state[0]), float(state[1])))
            max_points = max(50, int(self.node.history_sec * self.node.refresh_hz))
            self.actual_xy = self.actual_xy[-max_points:]
            actual = np.asarray(self.actual_xy)
            self.actual_line.set_data(actual[:, 0], actual[:, 1])
            self.vehicle_point.set_data([state[0]], [state[1]])
            arrow_length = 0.08
            self.heading_line.set_data(
                [state[0], state[0] + arrow_length * math.cos(state[2])],
                [state[1], state[1] + arrow_length * math.sin(state[2])],
            )

        prediction = np.asarray(snapshot.get("predicted_states", []), dtype=float)
        if prediction.size:
            self.prediction_line.set_data(prediction[:, 0], prediction[:, 1])
        else:
            self.prediction_line.set_data([], [])
        references = np.asarray(snapshot.get("reference_horizon", []), dtype=float)
        if references.size:
            self.reference_line.set_data(references[:, 0], references[:, 1])
        else:
            self.reference_line.set_data([], [])

        dt_sec = max(1e-6, float(snapshot.get("dt_sec", 0.10)))
        speed_util = self._safe_ratio(float(command[0]), float(limits[0]))
        curvature_util = self._safe_ratio(float(command[2]), float(limits[1]))
        angular_util = self._safe_ratio(float(command[1]), float(limits[2]))
        solve_util = float(solver[0]) / dt_sec
        x_values = np.asarray(
            [
                0.0 if state is None else float(state[0]),
                0.0 if state is None else float(state[1]),
                0.0 if state is None else float(state[2]),
                float(errors[0]),
                float(errors[1]),
            ]
        )
        self.data.update(
            _time=elapsed,
            _x=x_values,
            _u=np.asarray(command, dtype=float),
            _tvp=np.asarray(
                [
                    speed_util,
                    curvature_util,
                    angular_util,
                    solve_util,
                    float(solver[1]),
                    float(snapshot.get("progress_index", 0)),
                ]
            ),
        )
        self.graphics.plot_results()
        self.graphics.reset_axes()
        start_time = max(0.0, elapsed - self.node.history_sec)
        for axis in (self.input_axis, self.error_axis, self.condition_axis):
            axis.set_xlim(start_time, max(self.node.history_sec, elapsed + 0.1))
        self.condition_axis.set_ylim(-0.05, max(1.2, solve_util * 1.1))

        obstacles = snapshot.get("obstacles", [None, None])
        self.status_text.set_text(
            f"{snapshot.get('status', 'UNKNOWN')} | index={snapshot.get('progress_index', 0)} "
            f"| gear={snapshot.get('direction', 0):+d} | cost={float(solver[1]):.3f} "
            f"| solve={float(solver[0]) * 1000.0:.1f} ms | rejoin={snapshot.get('forward_rejoin_active', False)} "
            f"| obstacle(front/rear)={obstacles[0]}/{obstacles[1]}"
        )
        self.path_axis.relim()
        self.path_axis.autoscale_view()
        self.figure.canvas.draw_idle()
        return ()

    def show(self) -> None:
        self.animation = FuncAnimation(
            self.figure,
            self.update,
            interval=1000.0 / self.node.refresh_hz,
            cache_frame_data=False,
        )
        plt.show()


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = MpcVisualizationNode()
    executor_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    executor_thread.start()
    try:
        MpcDashboard(node).show()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
