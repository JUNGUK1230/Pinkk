"""MyCobot 비동기 이동 명령 큐를 실행 전후에 확실히 비운다."""

from __future__ import annotations


def _required_method(robot: object, name: str):
    method = getattr(robot, name, None)
    if not callable(method):
        raise RuntimeError(f'pymycobot {name}() API가 없습니다')
    return method


def _require_success(name: str, response: object) -> None:
    if response not in (True, 1):
        raise RuntimeError(f'{name}() 실패 응답: {response!r}')


def prepare_command_queue(robot: object) -> None:
    """기존 이동을 정지·삭제하고 항상 최신 명령만 실행하도록 설정한다."""
    _require_success('stop', _required_method(robot, 'stop')())
    _require_success('clear_queue', _required_method(robot, 'clear_queue')())
    _require_success(
        'set_fresh_mode',
        _required_method(robot, 'set_fresh_mode')(1),
    )
    mode = _required_method(robot, 'get_fresh_mode')()
    if mode not in (True, 1):
        raise RuntimeError(f'fresh mode 확인 실패: get_fresh_mode()={mode!r}')


def stop_and_clear_command_queue(robot: object) -> None:
    """현재 이동을 정지하고 남아 있는 모든 이동 명령을 삭제한다."""
    _require_success('stop', _required_method(robot, 'stop')())
    _require_success('clear_queue', _required_method(robot, 'clear_queue')())
