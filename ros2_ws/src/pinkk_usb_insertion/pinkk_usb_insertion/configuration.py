"""ROS 노드에서 공유하는 YAML 설정 로더."""

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
