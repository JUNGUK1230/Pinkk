"""실제 로봇 PC의 mc 초기화 코드를 연결하는 adapter.

이 파일 하나만 현장 로봇 연결 방식에 맞게 수정하면 나머지 코드는 바꿀 필요가 없다.
"""

from typing import Any


def create_robot() -> Any:
    """get_coords()를 제공하는 기존 mc 객체를 생성해 반환한다.

    로봇 PC에서 이미 사용하는 초기화 코드를 아래에 옮긴다. 로봇 모델, serial 포트,
    baudrate를 이 노트북에서 추측하지 않기 위해 기본 구현은 의도적으로 중단한다.

    예시 구조(그대로 사용하지 말 것)::

        from pymycobot... import 실제_클래스
        mc = 실제_클래스(실제_포트, 실제_baudrate)
        return mc
    """
    raise NotImplementedError(
        "robot_adapter.py의 create_robot()에 로봇 PC에서 이미 사용하는 mc 초기화 코드를 입력하세요"
    )


def validate_robot_frames(robot: Any) -> None:
    """get_coords()가 base 기준 flange pose를 반환하도록 현재 설정을 검사한다.

    공식 API에서 get_reference_frame()은 0=base/1=tool이고,
    get_end_type()은 0=flange/1=tool이다. 지원되지 않는 구형 API라면 조용히
    진행하지 않고 사용자가 로봇 PC의 실제 라이브러리를 확인하도록 중단한다.
    """
    from . import config

    if not callable(getattr(robot, "get_reference_frame", None)):
        raise RuntimeError("로봇 API에 get_reference_frame()이 없어 base 기준 여부를 확인할 수 없습니다")
    if not callable(getattr(robot, "get_end_type", None)):
        raise RuntimeError("로봇 API에 get_end_type()이 없어 flange 기준 여부를 확인할 수 없습니다")
    reference_frame = robot.get_reference_frame()
    end_type = robot.get_end_type()
    if reference_frame != config.EXPECTED_REFERENCE_FRAME:
        raise RuntimeError(
            f"reference frame={reference_frame}; Hand-Eye에는 base 기준(0)이 필요합니다"
        )
    if end_type != config.EXPECTED_END_TYPE:
        raise RuntimeError(f"end type={end_type}; Hand-Eye에는 flange 기준(0)이 필요합니다")
    print("로봇 좌표계 확인 완료: reference=base(0), end=flange(0)")
