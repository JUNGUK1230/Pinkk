"""Latch an emergency stop on Pinky and show a Korean LCD warning."""

from __future__ import annotations

import os
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool


FONT_CANDIDATES = (
    "~/pinky_lcd/example/MaruBuri-Bold.ttf",
    "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
)


class PinkyEmergencyLcd(Node):
    def __init__(self) -> None:
        super().__init__("pinky_emergency_lcd")
        self.declare_parameter("cmd_vel_topic", "/pinky1/cmd_vel")
        self.declare_parameter(
            "service_name", "/pinky1/set_emergency_stop"
        )
        self.declare_parameter(
            "state_topic", "/pinky1/emergency_stop_state"
        )
        self.declare_parameter("lcd_status_topic", "/pinky1/lcd_status")
        self.declare_parameter(
            "battery_percent_topic", "/pinky1/battery/percent"
        )
        self.declare_parameter("stop_publish_hz", 20.0)
        self.declare_parameter("lcd_backlight", 80)

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        service_name = str(self.get_parameter("service_name").value)
        state_topic = str(self.get_parameter("state_topic").value)
        lcd_status_topic = str(self.get_parameter("lcd_status_topic").value)
        battery_percent_topic = str(
            self.get_parameter("battery_percent_topic").value
        )
        publish_hz = float(self.get_parameter("stop_publish_hz").value)
        if publish_hz <= 0.0:
            raise ValueError("stop_publish_hz must be positive")

        self._latched = False
        self._cmd_vel_topic = cmd_vel_topic
        self._command_publisher = None
        self._battery_percent = None
        self._status_until = 0.0
        self._persistent_status = False
        self._last_lcd_key = None
        self._lcd = self._create_lcd()
        self._font_path = self._find_korean_font()
        self._service = self.create_service(
            SetBool, service_name, self._set_emergency_stop
        )
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._state_publisher = self.create_publisher(Bool, state_topic, state_qos)
        self._lcd_status_subscription = self.create_subscription(
            String, lcd_status_topic, self._lcd_status_callback, 10
        )
        self._battery_subscription = self.create_subscription(
            Float32, battery_percent_topic, self._battery_callback, 10
        )
        self._publish_state()
        self._timer = self.create_timer(1.0 / publish_hz, self._publish_stop)
        self.get_logger().info(
            f"Emergency stop service ready: {service_name} -> {cmd_vel_topic}"
        )

    def _create_lcd(self):
        try:
            from pinky_lcd import LCD
        except ImportError as error:
            raise RuntimeError(
                "pinky_lcd is required; run this node on Pinky Pro"
            ) from error
        lcd = LCD()
        lcd.set_backlight(int(self.get_parameter("lcd_backlight").value))
        return lcd

    @staticmethod
    def _find_korean_font() -> Path:
        configured = os.environ.get("PINKY_LCD_FONT")
        candidates = ((configured,) if configured else ()) + FONT_CANDIDATES
        for candidate in candidates:
            path = Path(candidate).expanduser()
            if path.is_file():
                return path
        raise RuntimeError(
            "Korean font not found. Set PINKY_LCD_FONT to a Korean TTF/OTF file."
        )

    def _set_emergency_stop(self, request, response):
        self._latched = bool(request.data)
        if self._latched:
            if self._command_publisher is None:
                self._command_publisher = self.create_publisher(
                    Twist, self._cmd_vel_topic, 10
                )
            self._command_publisher.publish(Twist())
            self._persistent_status = True
            self._show_text("긴급정지", (255, 0, 0), 62, "emergency")
            response.message = "긴급정지가 유지됩니다."
            self.get_logger().error("EMERGENCY STOP LATCHED")
        else:
            if self._command_publisher is not None:
                self._command_publisher.publish(Twist())
                self.destroy_publisher(self._command_publisher)
                self._command_publisher = None
            # 경로 재생성 명령이 바로 이어서 LCD 상태를 갱신한다. 여기서
            # "정상"을 잠깐 표시하지 않고 1초 동안 다음 상태를 기다린다.
            self._persistent_status = False
            self._status_until = time.monotonic() + 1.0
            self._last_lcd_key = None
            response.message = "긴급정지가 해제되었습니다."
            self.get_logger().warning("Emergency stop released")
        response.success = True
        self._publish_state()
        return response

    def _publish_state(self) -> None:
        message = Bool()
        message.data = self._latched
        self._state_publisher.publish(message)

    def _lcd_status_callback(self, message: String) -> None:
        if self._latched:
            return
        text = message.data.strip()
        styles = {
            "경로 생성 중": ((80, 180, 255), 46),
            "입차 중": ((255, 255, 255), 58),
            "출차 중": ((255, 255, 255), 58),
            "충전 중": ((80, 220, 120), 58),
            "일시 정지": ((255, 190, 0), 52),
        }
        if text not in styles:
            return
        self._persistent_status = text == "일시 정지"
        self._status_until = (
            float("inf")
            if self._persistent_status
            else time.monotonic() + 5.0
        )
        color, size = styles[text]
        self._show_text(text, color, size, f"status:{text}")

    def _battery_callback(self, message: Float32) -> None:
        self._battery_percent = max(0.0, min(100.0, float(message.data)))
        self._show_battery_if_due()

    def _show_battery_if_due(self) -> None:
        if (
            self._latched
            or self._persistent_status
            or time.monotonic() < self._status_until
            or self._battery_percent is None
        ):
            return
        rounded = round(self._battery_percent, 1)
        self._show_text(
            f"배터리 {rounded:.1f}%",
            (255, 255, 255),
            42,
            f"battery:{rounded:.1f}",
        )

    def _publish_stop(self) -> None:
        if self._latched and self._command_publisher is not None:
            self._command_publisher.publish(Twist())
        else:
            self._show_battery_if_due()

    def _show_text(
        self,
        text: str,
        color: tuple[int, int, int],
        font_size: int,
        lcd_key: str,
    ) -> None:
        from PIL import Image, ImageDraw, ImageFont

        if self._last_lcd_key == lcd_key:
            return
        image = Image.new("RGB", (320, 240), color=(0, 0, 0))
        draw = ImageDraw.Draw(image)
        font = ImageFont.truetype(str(self._font_path), font_size)
        bbox = draw.textbbox((0, 0), text, font=font)
        x = (320 - (bbox[2] - bbox[0])) // 2
        y = (240 - (bbox[3] - bbox[1])) // 2 - bbox[1]
        draw.text((x, y), text, fill=color, font=font)
        self._lcd.img_show(image)
        self._last_lcd_key = lcd_key

    def destroy_node(self) -> bool:
        if self._latched and self._command_publisher is not None:
            self._command_publisher.publish(Twist())
            self.destroy_publisher(self._command_publisher)
            self._command_publisher = None
        self._lcd.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PinkyEmergencyLcd()
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
