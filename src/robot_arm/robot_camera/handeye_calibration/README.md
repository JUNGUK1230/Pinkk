# 로봇 PC용 Eye-in-Hand Hand-Eye 캘리브레이션

이 코드는 먼저 노트북의 Git 저장소에서 작성한 뒤 로봇 PC로 가져가 SSH 환경에서
실행합니다. 실행 시 카메라는 Flask가 아니라 로봇 PC의 `cv2.VideoCapture()`로 직접
열고, 로봇 pose는 기존 `mc.get_coords()`에서 읽습니다.

## 좌표계

```text
p_A = T_A_B @ p_B

T_base_flange    : flange 좌표 -> base 좌표
T_camera_charuco : ChArUco 좌표 -> camera 좌표
T_flange_camera  : camera 좌표 -> flange 좌표 (최종 결과)

T_base_charuco =
    T_base_flange @ T_flange_camera @ T_camera_charuco
```

OpenCV에서는 `gripper2base=T_base_flange`, `target2cam=T_camera_charuco`,
반환되는 `cam2gripper=T_flange_camera`입니다.

## Git에 포함되는 것과 포함되지 않는 것

코드는 Git에 포함됩니다. 다음 측정 데이터와 결과는 `.gitignore`로 제외됩니다.

```text
camera_calibration/results/*
handeye_calibration/data/*
```

따라서 로봇 PC에는 현재 계산한 `intrinsics.npz`를 다음 위치로 별도 복사해야 합니다.

```text
src/robot_arm/robot_camera/camera_calibration/results/intrinsics.npz
```

현재 intrinsic은 `640×480`, checkerboard square `28 mm`, RMS `0.345252 px`입니다.

## 로봇 PC에서 준비할 내용

1. [config.py](config.py)의 ChArUco square/marker 실측값을 수정합니다.
2. [robot_adapter.py](robot_adapter.py)의 `create_robot()`에 기존 mc 초기화 코드를 넣습니다.
3. 수집 시작 시 코드가 `get_reference_frame()==0`(base)을 검사합니다.
4. 수집 시작 시 코드가 `get_end_type()==0`(flange)을 검사합니다.
5. 실제 로봇 pose를 읽어 값의 단위와 방향을 마지막으로 교차 확인합니다.

저장소 밖의 기존 테스트 파일이나 임시 변환 코드는 근거로 사용하지 않습니다. 회전은
Elephant Robotics 공식 문서의 다음 정의를 반영했습니다.

```python
rx, ry, rz = roll, pitch, yaw
rotation = Rotation.from_euler("ZYX", [rz, ry, rx], degrees=True)
```

즉 local/body 좌표계 기준 intrinsic ZYX 회전입니다. 로봇 PC의 API가
`get_reference_frame()` 또는 `get_end_type()`을 지원하지 않으면 좌표계를 확인할 수
없다는 오류로 중단하며, 이때는 설치된 pymycobot 버전과 제조사 API를 다시 확인합니다.

## 설치

```bash
cd ~/Pinkk-robot-arm
source ~/venv/mycobot/bin/activate
pip install -r requirements.txt
```

`opencv-contrib-python`, NumPy, SciPy가 필요합니다. 아래 결과가 모두 True여야 합니다.

```bash
python3 -c "import cv2; print(hasattr(cv2,'aruco')); print(hasattr(cv2,'calibrateHandEye'))"
```

## 1. Sample 수집

ChArUco 보드를 완전히 고정하고 로봇을 여러 위치와 회전 자세로 이동합니다.

```bash
python3 -m src.robot_arm.robot_camera.handeye_calibration.collect_handeye_samples \
  --camera 0
```

```text
S 또는 Space : 현재 sample 저장
Q 또는 ESC   : 종료
```

로봇을 완전히 정지한 뒤 저장합니다. 정면, 좌/우, 앞/뒤, 대각선 기울임을 포함한
15~30개 sample을 권장합니다. 이동만 반복하지 말고 회전 다양성을 확보해야 합니다.

## 2. 계산 및 방법 비교

```bash
python3 -m src.robot_arm.robot_camera.handeye_calibration.calibrate_handeye
```

TSAI, PARK, HORAUD, ANDREFF, DANIILIDIS를 모두 계산합니다. 고정 보드의 base pose
일관성을 위치 평균/최대[mm]와 회전 평균/최대[deg]로 출력합니다. 값이 작을수록 좋습니다.
특정 방법을 저장하려면 `--method PARK`처럼 지정합니다.

```text
data/T_flange_camera.npy
data/handeye_result.npz
```

## 3. 실시간 검증

```bash
python3 -m src.robot_arm.robot_camera.handeye_calibration.verify_handeye \
  --camera 0
```

보드를 고정한 상태에서 로봇 자세를 변경해도 화면의 Board Base X/Y/Z가 거의 일정해야
합니다. 크게 변하면 Euler convention, 보드 실측값, 카메라 해상도, 로봇 정지 여부,
카메라 고정 강성, sample 회전 다양성을 다시 확인합니다.
