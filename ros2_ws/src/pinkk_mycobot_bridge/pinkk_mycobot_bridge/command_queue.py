"""MyCobot 비동기 이동 명령 큐를 실행 전후에 확실히 비운다."""

from __future__ import annotations

import time


def _required_method(robot: object, name: str):
    method = getattr(robot, name, None)
    if not callable(method):
        raise RuntimeError(f'pymycobot {name}() API가 없습니다')
    return method


def require_no_explicit_command_failure(name: str, response: object) -> None:
    """일부 펌웨어의 무응답(-1/None)은 허용하되 명시적 실패는 거부한다."""
    if response not in (None, -1, True, 1):
        raise RuntimeError(f'{name}() 실패 응답: {response!r}')


def _require_stopped(
    robot: object,
    *,
    attempts: int = 5,
    interval_seconds: float = 0.1,
) -> None:
    """응답 없는 stop 명령 대신 실제 이동 상태를 조회해 정지를 확인한다."""
    is_moving = _required_method(robot, 'is_moving')
    responses: list[object] = []
    for attempt in range(attempts):
        response = is_moving()
        responses.append(response)
        if response in (False, 0):
            return
        if attempt + 1 < attempts:
            time.sleep(interval_seconds)
    raise RuntimeError(
        f'stop 확인 실패: is_moving() 응답={responses!r}'
    )


def prepare_command_queue(robot: object) -> None:
    """기존 이동을 정지·삭제하고 항상 최신 명령만 실행하도록 설정한다."""
    # pymycobot의 stop(), clear_queue(), set_fresh_mode()는 펌웨어에 따라
    # 성공 응답을 반환하지 않는다. 큐 삭제를 요청하고 fresh mode와 실제
    # 정지 상태를 조회해서 안전 상태를 확인한다.
    _required_method(robot, 'stop')()
    require_no_explicit_command_failure(
        'clear_queue',
        _required_method(robot, 'clear_queue')(),
    )
    _required_method(robot, 'set_fresh_mode')(1)
    mode = _required_method(robot, 'get_fresh_mode')()
    if mode not in (True, 1):
        raise RuntimeError(f'fresh mode 확인 실패: get_fresh_mode()={mode!r}')
    # fresh mode 전환 뒤 stop을 최신 명령으로 다시 보내 잔류 명령을 덮는다.
    _required_method(robot, 'stop')()
    _require_stopped(robot)


def stop_and_clear_command_queue(robot: object) -> None:
    """현재 이동을 정지하고 남아 있는 모든 이동 명령을 삭제한다."""
    _required_method(robot, 'stop')()
    require_no_explicit_command_failure(
        'clear_queue',
        _required_method(robot, 'clear_queue')(),
    )
    _required_method(robot, 'stop')()
    _require_stopped(robot)
