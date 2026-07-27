"""SQLite-backed robot command queue and audit history."""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import sqlite3
import threading
import uuid
from typing import Any


ALLOWED_COMMANDS = {"entry", "exit", "emergency_stop"}


class CommandStore:
    def __init__(self, database_path: str | Path) -> None:
        self.database_path = Path(database_path)
        self.database_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_lock = threading.Lock()
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.database_path, timeout=5)
        connection.row_factory = sqlite3.Row
        return connection

    def _initialize(self) -> None:
        with self._init_lock, self._connect() as connection:
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS robot_commands (
                    command_id TEXT PRIMARY KEY,
                    robot_id TEXT NOT NULL,
                    command TEXT NOT NULL,
                    parameters TEXT NOT NULL,
                    state TEXT NOT NULL,
                    created_at TEXT NOT NULL,
                    delivered_at TEXT,
                    completed_at TEXT,
                    result TEXT
                )
                """
            )
            connection.execute(
                "CREATE INDEX IF NOT EXISTS idx_robot_commands_queue "
                "ON robot_commands(robot_id, state, created_at)"
            )

    def create(self, robot_id: str, command: str, parameters: dict[str, Any] | None = None) -> dict[str, Any]:
        now = datetime.now(timezone.utc).isoformat()
        record = {
            "command_id": uuid.uuid4().hex,
            "robot_id": robot_id,
            "command": command,
            "parameters": parameters or {},
            "state": "pending",
            "created_at": now,
            "delivered_at": None,
            "completed_at": None,
            "result": None,
        }
        with self._connect() as connection:
            connection.execute(
                "INSERT INTO robot_commands VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                (
                    record["command_id"], robot_id, command,
                    json.dumps(record["parameters"], ensure_ascii=False),
                    record["state"], now, None, None, None,
                ),
            )
        return record

    def claim_next(self, robot_id: str) -> dict[str, Any] | None:
        now = datetime.now(timezone.utc).isoformat()
        with self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            row = connection.execute(
                "SELECT * FROM robot_commands WHERE robot_id = ? AND state = 'pending' "
                "ORDER BY CASE WHEN command = 'emergency_stop' THEN 0 ELSE 1 END, created_at LIMIT 1",
                (robot_id,),
            ).fetchone()
            if row is None:
                return None
            connection.execute(
                "UPDATE robot_commands SET state = 'delivered', delivered_at = ? WHERE command_id = ?",
                (now, row["command_id"]),
            )
            updated = dict(row)
            updated.update(state="delivered", delivered_at=now)
            return self._decode(updated)

    def complete(self, robot_id: str, command_id: str, state: str, result: dict[str, Any]) -> dict[str, Any] | None:
        now = datetime.now(timezone.utc).isoformat()
        with self._connect() as connection:
            cursor = connection.execute(
                "UPDATE robot_commands SET state = ?, completed_at = ?, result = ? "
                "WHERE command_id = ? AND robot_id = ? AND state = 'delivered'",
                (state, now, json.dumps(result, ensure_ascii=False), command_id, robot_id),
            )
            if cursor.rowcount == 0:
                return None
            row = connection.execute(
                "SELECT * FROM robot_commands WHERE command_id = ?", (command_id,)
            ).fetchone()
            return self._decode(dict(row))

    def history(self, limit: int = 50) -> list[dict[str, Any]]:
        with self._connect() as connection:
            rows = connection.execute(
                "SELECT * FROM robot_commands ORDER BY created_at DESC LIMIT ?", (limit,)
            ).fetchall()
        return [self._decode(dict(row)) for row in rows]

    @staticmethod
    def _decode(record: dict[str, Any]) -> dict[str, Any]:
        record["parameters"] = json.loads(record["parameters"])
        record["result"] = json.loads(record["result"]) if record["result"] else None
        return record
