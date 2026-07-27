"""Send changing fake battery values without requiring ROS 2."""

from datetime import datetime, timezone
import logging
import time

from config import Config
from http_sender import StatusSender


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    config = Config.from_env()
    sender = StatusSender(config)
    battery = 85.0
    while True:
        payload = {
            "robot_id": config.robot_id,
            "battery_percent": battery,
            "battery_voltage": round(11.0 + battery * 0.015, 2),
            "status": config.robot_status,
            "charging": config.robot_status == "charging",
            "timestamp": datetime.now(timezone.utc).astimezone().isoformat(),
        }
        sender.send(payload)
        battery = 100.0 if battery <= 5.0 else battery - 1.0
        time.sleep(config.send_interval)


if __name__ == "__main__":
    main()

