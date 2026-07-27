"""Reusable HTTP telemetry sender with bounded exponential backoff."""

from __future__ import annotations

import logging
import time
from typing import Any

import requests

from config import Config


LOGGER = logging.getLogger(__name__)


class StatusSender:
    def __init__(self, config: Config) -> None:
        self.config = config
        self.session = requests.Session()

    def send(self, payload: dict[str, Any], attempts: int = 3) -> bool:
        endpoint = f"{self.config.server_url}/api/robots/status"
        for attempt in range(attempts):
            try:
                response = self.session.post(
                    endpoint,
                    json=payload,
                    headers={"X-API-Key": self.config.api_key},
                    timeout=self.config.request_timeout,
                )
                response.raise_for_status()
                return True
            except requests.RequestException as error:
                LOGGER.warning("상태 전송 실패 (%d/%d): %s", attempt + 1, attempts, error)
                if attempt + 1 < attempts:
                    time.sleep(min(2 ** attempt, 4))
        return False

    def get_next_command(self) -> dict[str, Any] | None:
        endpoint = f"{self.config.server_url}/api/robots/{self.config.robot_id}/commands/next"
        try:
            response = self.session.get(
                endpoint,
                headers={"X-API-Key": self.config.api_key},
                timeout=self.config.request_timeout,
            )
            response.raise_for_status()
            return None if response.status_code == 204 else response.json()
        except requests.RequestException as error:
            LOGGER.warning("명령 조회 실패: %s", error)
            return None

    def report_command_result(self, command_id: str, state: str, message: str) -> bool:
        endpoint = (
            f"{self.config.server_url}/api/robots/{self.config.robot_id}"
            f"/commands/{command_id}/result"
        )
        try:
            response = self.session.post(
                endpoint,
                json={"state": state, "message": message},
                headers={"X-API-Key": self.config.api_key},
                timeout=self.config.request_timeout,
            )
            response.raise_for_status()
            return True
        except requests.RequestException as error:
            LOGGER.warning("명령 결과 전송 실패: %s", error)
            return False
