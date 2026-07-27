"""Environment-backed configuration for a PinkyPro telemetry client."""

from __future__ import annotations

from dataclasses import dataclass
import os


def _required(name: str) -> str:
    value = os.getenv(name, "").strip()
    if not value:
        raise RuntimeError(f"필수 환경변수 {name}가 설정되지 않았습니다.")
    return value


@dataclass(frozen=True)
class Config:
    robot_id: str
    server_url: str
    api_key: str
    percent_topic: str
    voltage_topic: str
    command_topic: str
    send_interval: float
    command_poll_interval: float
    request_timeout: float
    robot_status: str

    @classmethod
    def from_env(cls) -> "Config":
        interval = float(os.getenv("PINKK_SEND_INTERVAL", "1.5"))
        timeout = float(os.getenv("PINKK_HTTP_TIMEOUT", "3.0"))
        command_poll_interval = float(os.getenv("PINKK_COMMAND_POLL_INTERVAL", "1.0"))
        if interval <= 0 or timeout <= 0 or command_poll_interval <= 0:
            raise RuntimeError("전송 주기와 HTTP 타임아웃은 0보다 커야 합니다.")
        return cls(
            robot_id=_required("ROBOT_ID"),
            server_url=_required("CENTRAL_SERVER_URL").rstrip("/"),
            api_key=_required("PINKK_API_KEY"),
            percent_topic=os.getenv("PINKK_BATTERY_PERCENT_TOPIC", "/battery/percent"),
            voltage_topic=os.getenv("PINKK_BATTERY_VOLTAGE_TOPIC", "/battery/voltage"),
            command_topic=os.getenv("PINKK_COMMAND_TOPIC", "/pinkk/command"),
            send_interval=interval,
            command_poll_interval=command_poll_interval,
            request_timeout=timeout,
            robot_status=os.getenv("PINKK_ROBOT_STATUS", "idle"),
        )
