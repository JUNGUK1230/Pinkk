"""MyCobot 이동 큐 안전 처리를 검증한다."""

from pinkk_mycobot_bridge.command_queue import (
    command_gripper_value,
    prepare_command_queue,
    require_no_explicit_command_failure,
    stop_and_clear_command_queue,
)

import pytest


class FakeRobot:
    """호출 순서와 응답을 기록하는 최소 MyCobot 대역이다."""

    def __init__(
        self,
        *,
        fresh_mode=1,
        clear_response=1,
        moving_responses=None,
    ) -> None:
        """가짜 펌웨어 응답과 호출 기록을 초기화한다."""
        self.calls = []
        self.fresh_mode = fresh_mode
        self.clear_response = clear_response
        self.moving_responses = list(moving_responses or [0])

    def stop(self):
        """정지 명령을 기록한다."""
        self.calls.append(('stop',))
        return -1

    def clear_queue(self):
        """큐 삭제 명령과 설정된 응답을 반환한다."""
        self.calls.append(('clear_queue',))
        return self.clear_response

    def set_fresh_mode(self, mode):
        """Fresh mode 설정 명령을 기록한다."""
        self.calls.append(('set_fresh_mode', mode))
        return -1

    def get_fresh_mode(self):
        """현재 fresh mode 설정값을 반환한다."""
        self.calls.append(('get_fresh_mode',))
        return self.fresh_mode

    def is_moving(self):
        """순서대로 설정된 이동 상태를 반환한다."""
        self.calls.append(('is_moving',))
        if len(self.moving_responses) > 1:
            return self.moving_responses.pop(0)
        return self.moving_responses[0]

    def set_gripper_value(self, value, speed):
        """그리퍼 명령을 기록하고 무응답을 모의한다."""
        self.calls.append(('set_gripper_value', value, speed))
        return -1


def test_prepare_stops_clears_and_enables_fresh_mode() -> None:
    """이동 전 기존 큐를 비우고 fresh mode를 확인한다."""
    robot = FakeRobot()

    prepare_command_queue(robot)

    assert robot.calls == [
        ('stop',),
        ('clear_queue',),
        ('set_fresh_mode', 1),
        ('get_fresh_mode',),
        ('stop',),
        ('is_moving',),
    ]


def test_stop_and_clear_removes_pending_commands() -> None:
    """이동 종료 시 정지 뒤 큐를 삭제한다."""
    robot = FakeRobot()

    stop_and_clear_command_queue(robot)

    assert robot.calls == [
        ('stop',),
        ('clear_queue',),
        ('stop',),
        ('is_moving',),
    ]


def test_rejects_failed_clear_queue_response() -> None:
    """명시적인 큐 삭제 실패를 성공으로 처리하지 않는다."""
    robot = FakeRobot(clear_response=0)

    with pytest.raises(RuntimeError, match='clear_queue'):
        prepare_command_queue(robot)


def test_accepts_unconfirmed_clear_queue_with_state_verification() -> None:
    """응답 없는 펌웨어도 fresh mode와 정지를 확인하면 사용할 수 있다."""
    robot = FakeRobot(clear_response=-1)

    prepare_command_queue(robot)

    assert robot.calls[-2:] == [('stop',), ('is_moving',)]


def test_rejects_unconfirmed_fresh_mode() -> None:
    """Fresh mode를 읽어서 확인할 수 없으면 실행을 막는다."""
    robot = FakeRobot(fresh_mode=0)

    with pytest.raises(RuntimeError, match='fresh mode'):
        prepare_command_queue(robot)


@pytest.mark.parametrize('fresh_mode', [-1, None])
def test_allows_unconfirmed_fresh_mode_for_compatible_firmware(
    fresh_mode,
) -> None:
    """호환 옵션은 응답 없는 조회만 허용하고 실제 정지는 확인한다."""
    robot = FakeRobot(fresh_mode=fresh_mode)

    confirmed = prepare_command_queue(
        robot,
        allow_unconfirmed_fresh_mode=True,
    )

    assert confirmed is False
    assert robot.calls[-2:] == [('stop',), ('is_moving',)]


def test_compatibility_mode_still_rejects_disabled_fresh_mode() -> None:
    """호환 옵션에서도 명시적인 fresh mode 비활성 응답은 거부한다."""
    robot = FakeRobot(fresh_mode=0)

    with pytest.raises(RuntimeError, match='fresh mode'):
        prepare_command_queue(
            robot,
            allow_unconfirmed_fresh_mode=True,
        )


def test_waits_until_robot_reports_stopped() -> None:
    """Stop 응답이 없어도 실제 이동 상태가 0이 되면 성공한다."""
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


@pytest.mark.parametrize('response', [None, -1, True, 1])
def test_accepts_command_responses_verified_by_later_state(response) -> None:
    """명령 응답이 없어도 후속 위치·정지 감시가 판정할 수 있게 한다."""
    require_no_explicit_command_failure('send_angles', response)


def test_rejects_explicit_command_failure() -> None:
    """명시적인 실패 응답은 후속 상태 감시로 넘기지 않는다."""
    with pytest.raises(RuntimeError, match='send_angles'):
        require_no_explicit_command_failure('send_angles', 0)


def test_commands_gripper_value() -> None:
    """유효한 그리퍼 값과 속도를 한 번 전달한다."""
    robot = FakeRobot()

    response = command_gripper_value(robot, 10, 20)

    assert response == -1
    assert robot.calls == [('set_gripper_value', 10, 20)]


@pytest.mark.parametrize(('value', 'speed'), [(-1, 20), (101, 20), (10, 0)])
def test_rejects_invalid_gripper_command(value, speed) -> None:
    """허용 범위 밖의 그리퍼 명령을 거부한다."""
    with pytest.raises(ValueError):
        command_gripper_value(FakeRobot(), value, speed)
