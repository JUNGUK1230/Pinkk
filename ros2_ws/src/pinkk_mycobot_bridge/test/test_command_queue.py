"""MyCobot 이동 큐 안전 처리를 검증한다."""

import pytest

from pinkk_mycobot_bridge.command_queue import (
    prepare_command_queue,
    stop_and_clear_command_queue,
)


class FakeRobot:
    """호출 순서와 응답을 기록하는 최소 MyCobot 대역이다."""

    def __init__(
        self,
        *,
        fresh_mode=1,
        clear_response=1,
        moving_responses=None,
    ) -> None:
        self.calls = []
        self.fresh_mode = fresh_mode
        self.clear_response = clear_response
        self.moving_responses = list(moving_responses or [0])

    def stop(self):
        self.calls.append(('stop',))
        return -1

    def clear_queue(self):
        self.calls.append(('clear_queue',))
        return self.clear_response

    def set_fresh_mode(self, mode):
        self.calls.append(('set_fresh_mode', mode))
        return -1

    def get_fresh_mode(self):
        self.calls.append(('get_fresh_mode',))
        return self.fresh_mode

    def is_moving(self):
        self.calls.append(('is_moving',))
        if len(self.moving_responses) > 1:
            return self.moving_responses.pop(0)
        return self.moving_responses[0]


def test_prepare_stops_clears_and_enables_fresh_mode() -> None:
    """이동 전 기존 큐를 비우고 fresh mode를 확인한다."""
    robot = FakeRobot()

    prepare_command_queue(robot)

    assert robot.calls == [
        ('stop',),
        ('clear_queue',),
        ('set_fresh_mode', 1),
        ('get_fresh_mode',),
        ('is_moving',),
    ]


def test_stop_and_clear_removes_pending_commands() -> None:
    """이동 종료 시 정지 뒤 큐를 삭제한다."""
    robot = FakeRobot()

    stop_and_clear_command_queue(robot)

    assert robot.calls == [('stop',), ('clear_queue',), ('is_moving',)]


def test_rejects_failed_clear_queue_response() -> None:
    """큐 삭제 실패를 성공으로 처리하지 않는다."""
    robot = FakeRobot(clear_response=0)

    with pytest.raises(RuntimeError, match='clear_queue'):
        prepare_command_queue(robot)


def test_rejects_unconfirmed_fresh_mode() -> None:
    """fresh mode를 읽어서 확인할 수 없으면 실행을 막는다."""
    robot = FakeRobot(fresh_mode=0)

    with pytest.raises(RuntimeError, match='fresh mode'):
        prepare_command_queue(robot)


def test_waits_until_robot_reports_stopped() -> None:
    """stop 응답이 없어도 실제 이동 상태가 0이 되면 성공한다."""
    robot = FakeRobot(moving_responses=[1, 1, 0])

    prepare_command_queue(robot)

    assert robot.calls[-3:] == [
        ('is_moving',),
        ('is_moving',),
        ('is_moving',),
    ]


def test_rejects_unconfirmed_stop() -> None:
    """계속 이동 중이거나 통신 오류이면 큐 준비를 성공 처리하지 않는다."""
    robot = FakeRobot(moving_responses=[-1])

    with pytest.raises(RuntimeError, match='stop 확인 실패'):
        prepare_command_queue(robot)
