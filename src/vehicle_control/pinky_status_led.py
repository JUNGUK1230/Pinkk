"""Show battery thirds and emergency-stop state on Pinky's WS281x LEDs."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool


YELLOW = (255, 190, 0)
ORANGE = (255, 70, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
OFF = (0, 0, 0)


def battery_color(percent: float) -> tuple[int, int, int]:
    """Map equal battery thirds to yellow, orange, and red."""
    bounded = max(0.0, min(100.0, float(percent)))
    if bounded >= 200.0 / 3.0:
        return YELLOW
    if bounded >= 100.0 / 3.0:
        return ORANGE
    return RED


class PinkyStatusLed(Node):
    def __init__(self) -> None:
        super().__init__("pinky_status_led")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("service_name", "set_emergency_stop")
        self.declare_parameter("state_topic", "emergency_stop_state")
        self.declare_parameter("battery_percent_topic", "battery/percent")
        self.declare_parameter("status_topic", "lcd_status")
        self.declare_parameter("stop_publish_hz", 20.0)
        self.declare_parameter("led_count", 8)
        self.declare_parameter("led_brightness", 64)

        publish_hz = float(self.get_parameter("stop_publish_hz").value)
        if publish_hz <= 0.0:
            raise ValueError("stop_publish_hz must be positive")

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._latched = False
        self._pause_latched = False
        self._blink_on = False
        self._charge_blink_steps = 0
        self._battery_percent: float | None = None
        self._last_color: tuple[int, int, int] | None = None
        self._command_publisher = None
        self._led = self._create_led()

        service_name = str(self.get_parameter("service_name").value)
        state_topic = str(self.get_parameter("state_topic").value)
        battery_topic = str(self.get_parameter("battery_percent_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        self._service = self.create_service(
            SetBool, service_name, self._set_emergency_stop
        )
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._state_publisher = self.create_publisher(Bool, state_topic, state_qos)
        self._battery_subscription = self.create_subscription(
            Float32, battery_topic, self._battery_callback, 10
        )
        self._status_subscription = self.create_subscription(
            String, status_topic, self._status_callback, 10
        )
        self._stop_timer = self.create_timer(1.0 / publish_hz, self._publish_stop)
        self._blink_timer = self.create_timer(0.4, self._update_emergency_blink)
        self._charge_blink_timer = self.create_timer(0.25, self._update_charge_blink)
        self._set_led(OFF)
        self._publish_state()
        self.get_logger().info(
            f"Status LED ready: {battery_topic}, {status_topic}; "
            f"emergency service: {service_name}"
        )

    def _create_led(self):
        try:
            from pinkylib import LED
        except ImportError as error:
            raise RuntimeError("pinkylib LED support is required on Pinky") from error
        return LED(
            num=int(self.get_parameter("led_count").value),
            brightness=int(self.get_parameter("led_brightness").value),
        )

    def _battery_callback(self, message: Float32) -> None:
        self._battery_percent = max(0.0, min(100.0, float(message.data)))
        if (
            not self._latched
            and not self._pause_latched
            and self._charge_blink_steps == 0
        ):
            color = battery_color(self._battery_percent)
            if color != self._last_color:
                label = {YELLOW: "yellow", ORANGE: "orange", RED: "red"}[color]
                self.get_logger().info(
                    f"Battery {self._battery_percent:.1f}% -> LED {label}"
                )
            self._set_led(color)

    def _status_callback(self, message: String) -> None:
        if self._latched:
            return
        status = message.data.strip()
        if status == "일시 정지":
            self._charge_blink_steps = 0
            self._pause_latched = True
            self._blink_on = True
            self._set_led(YELLOW)
            self.get_logger().warning("Pause requested -> LED yellow blinking")
            return
        if status not in {"경로 생성 중", "입차 중", "출차 중", "충전 중"}:
            return
        if self._pause_latched:
            self._pause_latched = False
            self._blink_on = False
        if status != "충전 중":
            self._show_battery()
            return
        # 즉시 초록색을 켠 뒤 OFF/ON/OFF 네 단계로 총 두 번 점멸한다.
        self._charge_blink_steps = 4
        self._set_led(GREEN)
        self.get_logger().info("Charging request -> LED green blink x2")

    def _set_emergency_stop(self, request, response):
        self._latched = bool(request.data)
        if self._latched:
            self._charge_blink_steps = 0
            if self._command_publisher is None:
                self._command_publisher = self.create_publisher(
                    Twist, self._cmd_vel_topic, 10
                )
            self._command_publisher.publish(Twist())
            self._blink_on = True
            self._set_led(RED)
            response.message = "긴급정지가 유지됩니다."
            self.get_logger().error("EMERGENCY STOP LATCHED")
        else:
            if self._command_publisher is not None:
                self._command_publisher.publish(Twist())
                self.destroy_publisher(self._command_publisher)
                self._command_publisher = None
            self._blink_on = False
            self._show_battery()
            response.message = "긴급정지가 해제되었습니다."
            self.get_logger().warning("Emergency stop released")
        response.success = True
        self._publish_state()
        return response

    def _publish_state(self) -> None:
        message = Bool()
        message.data = self._latched
        self._state_publisher.publish(message)

    def _publish_stop(self) -> None:
        if self._latched and self._command_publisher is not None:
            self._command_publisher.publish(Twist())

    def _update_emergency_blink(self) -> None:
        if not self._latched and not self._pause_latched:
            return
        self._blink_on = not self._blink_on
        color = RED if self._latched else YELLOW
        self._set_led(color if self._blink_on else OFF)

    def _update_charge_blink(self) -> None:
        if self._latched or self._charge_blink_steps == 0:
            return
        self._charge_blink_steps -= 1
        if self._charge_blink_steps == 0:
            self._show_battery()
            return
        self._set_led(OFF if self._charge_blink_steps % 2 else GREEN)

    def _show_battery(self) -> None:
        self._set_led(
            OFF if self._battery_percent is None else battery_color(self._battery_percent)
        )

    def _set_led(self, color: tuple[int, int, int]) -> None:
        if color == self._last_color:
            return
        self._led.fill(color)
        self._last_color = color

    def destroy_node(self) -> bool:
        if self._command_publisher is not None:
            self._command_publisher.publish(Twist())
            self.destroy_publisher(self._command_publisher)
            self._command_publisher = None
        self._led.clear()
        self._led.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PinkyStatusLed()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
