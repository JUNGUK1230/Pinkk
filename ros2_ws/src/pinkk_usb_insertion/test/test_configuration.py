from pathlib import Path

import pytest

from pinkk_usb_insertion.configuration import load_yaml


def test_load_yaml_returns_top_level_mapping(tmp_path: Path) -> None:
    config = tmp_path / 'config.yaml'
    config.write_text('port_model:\n  width_m: 0.018\n', encoding='utf-8')

    assert load_yaml(config) == {'port_model': {'width_m': 0.018}}


def test_load_yaml_rejects_non_mapping(tmp_path: Path) -> None:
    config = tmp_path / 'config.yaml'
    config.write_text('- one\n- two\n', encoding='utf-8')

    with pytest.raises(ValueError, match='mapping'):
        load_yaml(config)


def test_load_yaml_rejects_missing_file(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match='설정 파일이 없습니다'):
        load_yaml(tmp_path / 'missing.yaml')
