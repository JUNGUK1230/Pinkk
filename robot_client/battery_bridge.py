"""Subscribe to PinkyPro Float32 battery topics and forward them over HTTP."""

from __future__ import annotations

from datetime import datetime, timezone
import logging
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import String
import json

from config import Config
from http_sender import StatusSender


class BatteryBridge(Node):
    def __init__(self, config: Config) -> None:
        super().__init__(f"{config.robot_id}_battery_http_bridge")
        self.config = config
        self.sender = StatusSender(config)
        self._lock = threading.Lock()
        self._sending = False
        self._percent: float | None = None
        self._voltage: float | None = None
        self._command_polling = False
        self.create_subscription(Float32, config.percent_topic, self._on_percent, 10)
        self.create_subscription(Float32, config.voltage_topic, self._on_voltage, 10)
        self._command_publisher = self.create_publisher(String, config.command_topic, 10)
        self.create_timer(config.send_interval, self._schedule_send)
        self.create_timer(config.command_poll_interval, self._schedule_command_poll)

    def _on_percent(self, message: Float32) -> None:
        with self._lock:
            self._percent = max(0.0, min(100.0, float(message.data)))

    def _on_voltage(self, message: Float32) -> None:
        with self._lock:
            self._voltage = float(message.data)

    def _schedule_send(self) -> None:
        with self._lock:
            if self._percent is None or self._sending:
                return
            self._sending = True
            payload = {
                "robot_id": self.config.robot_id,
                "battery_percent": round(self._percent, 1),
                "battery_voltage": None if self._voltage is None else round(self._voltage, 2),
                "status": self.config.robot_status,
                "charging": self.config.robot_status == "charging",
                "timestamp": datetime.now(timezone.utc).astimezone().isoformat(),
            }
        threading.Thread(target=self._send, args=(payload,), daemon=True).start()

    def _send(self, payload: dict) -> None:
        try:
            self.sender.send(payload)
        finally:
            with self._lock:
                self._sending = False

    def _schedule_command_poll(self) -> None:
        with self._lock:
            if self._command_polling:
                return
            self._command_polling = True
        threading.Thread(target=self._poll_command, daemon=True).start()

    def _poll_command(self) -> None:
        try:
            command = self.sender.get_next_command()
            if command is None:
                return
            message = String()
            message.data = json.dumps(command, ensure_ascii=False)
            self._command_publisher.publish(message)
            self.get_logger().warning(
                f"중앙 명령을 {self.config.command_topic} 토픽에 전달: {command['command']}"
            )
            self.sender.report_command_result(
                command["command_id"], "completed", "ROS 명령 토픽에 전달됨"
            )
        except Exception as error:
            self.get_logger().error(f"중앙 명령 처리 실패: {error}")
        finally:
            with self._lock:
                self._command_polling = False


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    config = Config.from_env()
    rclpy.init()
    node = BatteryBridge(config)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
