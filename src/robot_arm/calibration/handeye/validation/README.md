# 연결 확인과 결과 검증

이 폴더는 로봇 연결 상태와 최종 Hand-Eye 결과를 검증할 때 사용합니다.

```text
verify.py  저장된 T_flange_camera로 고정 보드 pose 안정성 확인
```

## 이 단계의 위치

검증은 두 번 합니다. sample 수집 전에는 로봇 pose를 읽을 수 있는지 확인하고, 계산 후에는
저장된 `T_flange_camera`가 실제 움직임에서도 안정적인지 확인합니다.

## 이동 명령 없는 로봇 연결 검사

로봇 PC에서 캘리브레이션 전에 로봇 pose를 읽을 수 있는지 확인합니다. 이 검사는 pose만 읽고 로봇
이동 명령은 보내지 않습니다.

```bash
python3 -c "
from src.robot_arm.calibration.handeye.core.robot_adapter import (
    create_robot,
    validate_robot_frames,
)

mc = create_robot()
print('coords:', mc.get_coords())
print('reference frame:', mc.get_reference_frame())
print('end type:', mc.get_end_type())
validate_robot_frames(mc)
"
```

정상 목표는 `reference frame=0`(base), `end type=0`(flange)입니다.

```text
로봇 연결 시도: MyCobot280(port='/dev/ttyUSB0', baud=1000000)
coords: [x, y, z, rx, ry, rz]
reference frame: 0
end type: 0
로봇 좌표계 확인 완료: reference=base(0), end=flange(0)
```

## 연결 오류 판단

| 결과 | 의미 | 조치 |
|---|---|---|
| `No such file or directory` | 지정한 device가 없음 | `list_ports`로 경로 재검색 |
| `Permission denied` | 사용자에게 serial 권한이 없음 | `dialout` group 확인 |
| `Device or resource busy` | 다른 프로세스가 port 점유 | `fuser`로 프로세스 확인 |
| `coords=None` 또는 timeout | 장치는 열렸지만 로봇 응답이 아님 | 로봇 연결, class, baudrate 확인 |
| `reference frame=1` | tool 기준 pose | base 기준 설정 후 재확인 |
| `end type=1` | tool 끝 기준 pose | flange 기준 설정 후 재확인 |
| 모든 값 정상 | `T_base_flange` 수집 가능 | ChArUco 검출 단계 진행 |

로봇 포트임이 확인되기 전에는 `send_angles()`, `send_coords()` 같은 이동 명령을 실행하지
않습니다.

## 실시간 Hand-Eye 결과 검증

sample 계산 후 다음 명령을 실행합니다.

```bash
python3 -m src.robot_arm.calibration.handeye.validation.verify \
  --camera 0
```

보드를 고정한 상태에서 로봇 자세를 바꾸어도 화면의 `Board Base X/Y/Z`가 거의 일정해야
합니다.

값이 크게 변하면 다음 항목을 다시 확인합니다.

1. 로봇 Euler convention
2. ChArUco square/marker 실측값
3. 카메라 해상도와 intrinsic 해상도
4. sample 저장 시 로봇 정지 여부
5. 카메라 고정 강성
6. sample 회전 다양성
