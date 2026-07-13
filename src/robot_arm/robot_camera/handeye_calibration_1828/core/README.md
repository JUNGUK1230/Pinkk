# 핵심 로직과 좌표계

이 폴더는 실행 파일에서 공통으로 사용하는 계산 로직을 모아둡니다.

## 파일 역할

```text
calibration.py     OpenCV Hand-Eye 계산과 결과 점수화
charuco.py         ChArUco 보드 검출과 T_camera_charuco 추정
io.py              카메라 열기, 해상도 확인, sample 저장/로드
robot_adapter.py   pymycobot 연결과 좌표계 확인
transforms.py      회전/이동 행렬 검증과 로봇 pose 변환
```

## 좌표계 규칙

```text
p_A = T_A_B @ p_B

T_base_flange    : flange 좌표 -> base 좌표
T_camera_charuco : ChArUco 좌표 -> camera 좌표
T_flange_camera  : camera 좌표 -> flange 좌표
```

최종 검증에 쓰는 고정 보드 pose는 다음처럼 계산합니다.

```text
T_base_charuco =
    T_base_flange @ T_flange_camera @ T_camera_charuco
```

로봇 자세를 바꾸어도 ChArUco 보드가 고정되어 있다면 `T_base_charuco`의 위치와 회전은
거의 일정해야 합니다.

## OpenCV Hand-Eye 입력

OpenCV `calibrateHandEye()`에는 다음 값을 넣습니다.

```text
gripper2base = T_base_flange
target2cam   = T_camera_charuco
```

OpenCV가 반환하는 `cam2gripper`는 이 프로젝트에서 원하는 `T_flange_camera`입니다.

## 로봇 Euler convention

`mc.get_coords()`의 `[x, y, z, rx, ry, rz]`에서 위치 단위는 `mm`, 각도 단위는 `degree`로
취급합니다. 회전은 Elephant Robotics 문서 기준으로 intrinsic ZYX를 사용합니다.

```python
rx, ry, rz = roll, pitch, yaw
rotation = Rotation.from_euler("ZYX", [rz, ry, rx], degrees=True)
```

`settings.py`의 `ROBOT_EULER_CONVENTION_VERIFIED`가 `False`이면 sample 수집을 중단합니다.
실제 로봇 PC에서 pose 방향을 확인한 뒤에만 `True`로 둡니다.
