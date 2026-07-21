"""YAML 설정 로딩과 실행 안전 조건 검사."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def load_yaml(path: str | Path) -> dict[str, Any]:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f'설정 파일이 없습니다: {config_path}')
    data = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError(f'YAML 최상위 항목은 mapping이어야 합니다: {config_path}')
    return data


def execution_gate(control: dict[str, Any], tool: dict[str, Any]) -> tuple[bool, str]:
    execution = control.get('execution', {})
    tool_data = tool.get('tool', {})
    if not bool(execution.get('execution_enabled', False)):
        return False, 'execution_enabled=false'
    if bool(execution.get('require_calibrated_tool', True)) and not bool(
        tool_data.get('calibrated', False)
    ):
        return False, 'tool.calibrated=false'
    return True, '실행 허용'


def insertion_gate(control: dict[str, Any], tool: dict[str, Any]) -> tuple[bool, str]:
    allowed, reason = execution_gate(control, tool)
    if not allowed:
        return allowed, reason
    if not bool(control.get('execution', {}).get('insertion_enabled', False)):
        return False, 'insertion_enabled=false'
    return True, '삽입 허용'
