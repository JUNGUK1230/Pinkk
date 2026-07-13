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

## 로봇 PC 환경 구성

아래 과정은 실제 로봇 PC에서 확인한 환경을 기준으로 작성했습니다.

```text
운영 환경: Linux aarch64
Python: 3.12.3
가상환경: ~/venv/mycobot
정상 확인 OpenCV: 4.12.0
정상 확인 NumPy: 2.2.6
정상 확인 SciPy: 1.18.0
```

### 1. 저장소의 development 브랜치 받기

로봇 PC에 저장소가 없다면 홈 디렉터리에서 clone합니다. 삭제된 디렉터리에 현재 shell이
남아 있으면 `Unable to read current working directory` 오류가 발생하므로 먼저 `cd ~`를
실행합니다.

```bash
cd ~

git clone \
  --branch development \
  --single-branch \
  https://github.com/JUNGUK1230/Pinkk.git \
  ~/Pinkk-robot-arm

cd ~/Pinkk-robot-arm
git branch --show-current
```

기존 저장소가 있다면 다음과 같이 갱신합니다.

```bash
cd ~/Pinkk-robot-arm
git fetch origin
git switch development
git pull --ff-only origin development
```

### 2. mycobot 가상환경 활성화

시스템 Python과 로봇 제어 환경을 섞지 않도록 기존 가상환경을 활성화합니다.

```bash
source ~/venv/mycobot/bin/activate
cd ~/Pinkk-robot-arm
```

terminal 앞에 `(mycobot)`이 표시되는지 확인합니다.

```text
(mycobot) jetcobot@raspi:~/Pinkk-robot-arm$
```

Python과 CPU architecture를 확인합니다.

```bash
python3 -c "
import platform
import sys

print('Python:', sys.version)
print('Architecture:', platform.machine())
"
```

실제 확인 환경은 Python 3.12.3, `aarch64`입니다.

### 3. 기존 Python 패키지 상태 백업

OpenCV를 교체하기 전에 현재 가상환경의 패키지 목록을 저장합니다.

```bash
python3 -m pip freeze > ~/mycobot_packages_before_handeye.txt
```

관련 패키지를 확인합니다.

```bash
python3 -m pip list | grep -Ei "opencv|numpy|scipy"
```

### 4. OpenCV 기능 사전 검사

Hand-Eye에는 `cv2.aruco`뿐 아니라 `cv2.calibrateHandEye`가 반드시 필요합니다.

```bash
python3 -c "
import cv2

print('OpenCV:', cv2.__version__)
print('cv2 path:', cv2.__file__)
print('ArUco:', hasattr(cv2, 'aruco'))
print('HandEye:', hasattr(cv2, 'calibrateHandEye'))
"
```

실제 로봇 PC에 처음 설치되어 있던 `opencv-python 5.0.0.93`에서는 다음 결과가
확인됐습니다.

```text
OpenCV: 5.0.0
ArUco: True
HandEye: False
```

이 상태에서는 ChArUco 검출은 가능해도 Hand-Eye 계산은 실행할 수 없습니다.

### 5. 충돌하는 OpenCV 제거

OpenCV Python 패키지들은 모두 같은 `cv2` namespace를 사용합니다. `opencv-python`과
`opencv-contrib-python`을 동시에 설치하지 말고 하나만 사용해야 합니다.

`(mycobot)` 환경이 활성화되어 있는지 다시 확인한 뒤 실행합니다.

```bash
python3 -m pip uninstall -y \
  opencv-python \
  opencv-python-headless \
  opencv-contrib-python \
  opencv-contrib-python-headless
```

설치되어 있지 않은 패키지에 `Skipping ... as it is not installed` 경고가 나오는 것은
정상입니다.

### 6. Hand-Eye 지원 OpenCV와 SciPy 설치

이 프로젝트는 Python 3.12/aarch64 wheel이 제공되고 `calibrateHandEye()` 동작을 확인한
버전으로 고정합니다.

```bash
python3 -m pip install --no-cache-dir \
  opencv-contrib-python==4.12.0.88 \
  scipy
```

또는 저장소의 고정된 의존성을 사용합니다.

```bash
python3 -m pip install -r requirements.txt
```

`opencv-contrib-python`은 main 모듈과 contrib/extra 모듈을 모두 포함하므로 별도의
`opencv-python`을 추가 설치하지 않습니다.

### 7. 설치 결과 최종 검사

다음 명령을 그대로 실행합니다. `__version__`, `__file__`에는 앞뒤로 underscore가
각각 두 개씩 들어갑니다.

```bash
python3 -c "
import cv2
import numpy
import scipy

print('OpenCV:', cv2.__version__)
print('cv2 path:', cv2.__file__)
print('NumPy:', numpy.__version__)
print('SciPy:', scipy.__version__)
print('ArUco:', hasattr(cv2, 'aruco'))
print('HandEye:', hasattr(cv2, 'calibrateHandEye'))
"
```

실제 정상 확인 결과는 다음과 같습니다.

```text
OpenCV: 4.12.0
NumPy: 2.2.6
SciPy: 1.18.0
ArUco: True
HandEye: True
```

`ArUco`와 `HandEye` 중 하나라도 False이면 sample 수집이나 계산을 시작하지 않습니다.

### 8. 카메라 장치 확인

먼저 Linux 카메라 장치를 확인합니다.

```bash
ls -l /dev/video*
```

OpenCV에서 읽을 수 있는 카메라 index를 검색합니다.

```bash
python3 \
  src/robot_arm/robot_camera/camera_calibration/scripts/test_opencv_camera.py \
  --scan
```

카메라가 0번이라면 내부 캘리브레이션과 동일한 640×480으로 확인합니다.

```bash
python3 \
  src/robot_arm/robot_camera/camera_calibration/scripts/test_opencv_camera.py \
  --camera 0 \
  --width 640 \
  --height 480
```

SSH에서 `cv2.imshow()` 창을 사용하려면 로봇 PC에 모니터가 연결되어 있거나 X11
forwarding이 필요합니다.

```bash
ssh -X jetcobot@로봇_PC_IP
```

### 9. 환경 복구 참고

패키지 교체 후 기존 로봇 프로그램에 문제가 생기면 먼저 저장해 둔 목록과 현재 목록을
비교합니다.

```bash
python3 -m pip freeze > ~/mycobot_packages_after_handeye.txt
diff ~/mycobot_packages_before_handeye.txt ~/mycobot_packages_after_handeye.txt
```

패키지 전체를 무조건 재설치하기 전에 OpenCV, NumPy, SciPy 차이와 오류 메시지를 먼저
확인합니다. 카메라 확인과 로봇 연결 확인이 모두 끝난 뒤에만 sample 수집으로 넘어갑니다.

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
