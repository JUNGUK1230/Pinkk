"""Thread-safe in-memory storage for the latest robot telemetry."""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import threading
from typing import Any


EXPECTED_ROBOTS = ("pinky1", "pinky2")
DELAYED_AFTER_SECONDS = 5.0
OFFLINE_AFTER_SECONDS = 10.0


class RobotStatusStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._robots: dict[str, dict[str, Any]] = {}

    def update(self, payload: dict[str, Any], received_at: datetime | None = None) -> dict[str, Any]:
        now = received_at or datetime.now(timezone.utc)
        record = {
            "robot_id": payload["robot_id"],
            "battery_percent": payload["battery_percent"],
            "battery_voltage": payload.get("battery_voltage"),
            "status": payload["status"],
            "charging": payload["charging"],
            "timestamp": payload.get("timestamp"),
            "last_updated": now.isoformat(),
            "_received_at": now,
        }
        with self._lock:
            self._robots[payload["robot_id"]] = record
        return self._public_record(record, now)

    def snapshot(self, now: datetime | None = None) -> dict[str, dict[str, Any]]:
        current = now or datetime.now(timezone.utc)
        with self._lock:
            records = deepcopy(self._robots)
        return {
            robot_id: self._public_record(records.get(robot_id), current)
            for robot_id in (*EXPECTED_ROBOTS, *records)
        }

    @staticmethod
    def _public_record(record: dict[str, Any] | None, now: datetime) -> dict[str, Any]:
        if record is None:
            return {
                "battery_percent": None,
                "battery_voltage": None,
                "status": "unknown",
                "charging": False,
                "last_updated": None,
                "age_seconds": None,
                "connection": "offline",
                "online": False,
            }
        age = max(0.0, (now - record["_received_at"]).total_seconds())
        connection = "online"
        if age >= OFFLINE_AFTER_SECONDS:
            connection = "offline"
        elif age >= DELAYED_AFTER_SECONDS:
            connection = "delayed"
        public = {key: value for key, value in record.items() if not key.startswith("_") and key != "robot_id"}
        public.update(age_seconds=round(age, 1), connection=connection, online=connection != "offline")
        return public

