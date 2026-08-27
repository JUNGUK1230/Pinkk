"""Display battery, driving status, and emergency state on Pinky's LCD."""

from __future__ import annotations

import os
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


FONT_CANDIDATES = (
    "~/pinky_lcd/example/MaruBuri-Bold.ttf",
    "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
)


class PinkyStatusLcd(Node):
    def __init__(self) -> None:
        super().__init__("pinky_status_lcd")
        self.declare_parameter("battery_percent_topic", "battery/percent")
        self.declare_parameter("lcd_status_topic", "lcd_status")
        self.declare_parameter("emergency_state_topic", "emergency_stop_state")
        self.declare_parameter("lcd_backlight", 80)

        self._battery_percent: float | None = None
        self._status_until = 0.0
        self._persistent_status = False
        self._emergency = False
        self._last_lcd_key: str | None = None
        self._lcd = self._create_lcd()
        self._font_path = self._find_korean_font()

        self._battery_subscription = self.create_subscription(
            Float32,
            str(self.get_parameter("battery_percent_topic").value),
            self._battery_callback,
            10,
        )
        self._status_subscription = self.create_subscription(
            String,
            str(self.get_parameter("lcd_status_topic").value),
            self._status_callback,
            10,
        )
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._emergency_subscription = self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_state_topic").value),
            self._emergency_callback,
            state_qos,
        )
        self._timer = self.create_timer(0.2, self._show_battery_if_due)
        self.get_logger().info("Status LCD ready")

    def _create_lcd(self):
        try:
            from pinky_lcd import LCD
        except ImportError as error:
            raise RuntimeError("pinky_lcd is required on Pinky") from error
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
        raise RuntimeError("Korean font not found; set PINKY_LCD_FONT")

    def _battery_callback(self, message: Float32) -> None:
        self._battery_percent = max(0.0, min(100.0, float(message.data)))
        self._show_battery_if_due()

    def _status_callback(self, message: String) -> None:
        if self._emergency:
            return
        text = message.data.strip()
        styles = {
            "경로 생성 중": ((80, 180, 255), 46),
            "입차 중": ((255, 255, 255), 58),
            "출차 중": ((255, 255, 255), 58),
            "충전 중": ((80, 220, 120), 58),
            "주차칸 이동 중": ((255, 255, 255), 44),
            "일시 정지": ((255, 190, 0), 52),
        }
        if text not in styles:
            return
        # 충전 상태는 다음 주행 상태 명령이 올 때까지 화면에 유지한다.
        self._persistent_status = text in {"충전 중", "일시 정지"}
        self._status_until = float("inf") if self._persistent_status else time.monotonic() + 5.0
        color, size = styles[text]
        self._show_text(text, color, size, f"status:{text}")

    def _emergency_callback(self, message: Bool) -> None:
        self._emergency = bool(message.data)
        if self._emergency:
            self._persistent_status = True
            self._show_text("긴급정지", (255, 0, 0), 62, "emergency")
        else:
            self._persistent_status = False
            self._status_until = 0.0
            self._last_lcd_key = None
            self._show_battery_if_due()

    def _show_battery_if_due(self) -> None:
        if (
            self._emergency
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
        self._clear_lcd()
        return super().destroy_node()

    def _clear_lcd(self) -> None:
        from PIL import Image

        try:
            self._lcd.img_show(Image.new("RGB", (320, 240), color=(0, 0, 0)))
            self._lcd.set_backlight(0)
        finally:
            self._lcd.close()


def clear_lcd_once() -> None:
    """남은 LCD 프로세스가 강제 종료된 경우에도 화면을 확실히 끈다."""
    from PIL import Image
    from pinky_lcd import LCD

    lcd = LCD()
    try:
        lcd.img_show(Image.new("RGB", (320, 240), color=(0, 0, 0)))
        lcd.set_backlight(0)
    finally:
        lcd.close()


def main() -> None:
    if "--clear-only" in os.sys.argv[1:]:
        clear_lcd_once()
        return
    rclpy.init()
    node = PinkyStatusLcd()
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
