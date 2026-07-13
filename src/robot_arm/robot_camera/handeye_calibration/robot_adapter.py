"""로봇 PC에서 MyCobot280 연결 객체를 생성하고 좌표계를 검사하는 adapter."""

import os
from pathlib import Path
from typing import Any


DEFAULT_ROBOT_PORT = "/dev/ttyUSB0"
DEFAULT_ROBOT_BAUD = 1_000_000


def create_robot() -> Any:
    """MyCobot280 serial 연결 객체를 생성해 반환한다.

    로봇 PC에서 발견된 유일한 serial 후보는 /dev/ttyUSB0이고 baud 후보는
    1,000,000이다. get_coords() 응답으로 로봇 포트임을 최종 확인해야 한다.
    다른 환경에서는 JETCOBOT_PORT와 JETCOBOT_BAUD 환경변수로 덮어쓸 수 있다.
    pymycobot import를 함수 안에서 수행하여 장비가 없는 개발 PC에서도 문서와
    명령행 도움말을 확인할 수 있게 한다.
    """
    from pymycobot import MyCobot280

    port = os.environ.get("JETCOBOT_PORT", DEFAULT_ROBOT_PORT)
    try:
        baud = int(os.environ.get("JETCOBOT_BAUD", str(DEFAULT_ROBOT_BAUD)))
    except ValueError as error:
        raise ValueError("JETCOBOT_BAUD는 정수여야 합니다") from error
    if not Path(port).exists():
        raise FileNotFoundError(
            f"로봇 serial 장치가 없습니다: {port}. "
            "python3 -m serial.tools.list_ports -v로 실제 포트를 확인하세요"
        )
    print(f"로봇 연결 시도: MyCobot280(port={port!r}, baud={baud})")
    return MyCobot280(port, baud)


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
