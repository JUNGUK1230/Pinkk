# 환경설정과 로봇 PC 준비

이 폴더는 Hand-Eye 캘리브레이션 실행 전에 확인해야 하는 설정을 모아둡니다.
실제 상수는 [settings.py](settings.py)에서 관리합니다.

## 먼저 확인할 값

1. ChArUco 보드의 square 길이와 marker 길이가 실측값과 맞는지 확인합니다.
2. 카메라 해상도가 intrinsic을 계산한 해상도와 같은지 확인합니다.
3. 로봇 serial port와 baudrate가 실제 로봇 PC 환경과 맞는지 확인합니다.
4. `get_reference_frame()==0`이면 base 기준 pose입니다.
5. `get_end_type()==0`이면 flange 기준 pose입니다.

## 로봇 PC 환경

아래 조합에서 동작을 확인했습니다.

```text
운영 환경: Linux aarch64
Python: 3.12.3
가상환경: ~/venv/mycobot
OpenCV: 4.12.0
NumPy: 2.2.6
SciPy: 1.18.0
```

기본 도구가 없다면 먼저 설치합니다.

```bash
sudo apt update
sudo apt install -y git python3 python3-pip python3-venv v4l-utils lsof
```

저장소가 없다면 홈 디렉터리에서 clone합니다.

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

기존 저장소가 있다면 최신 상태로 갱신합니다.

```bash
cd ~/Pinkk-robot-arm
git fetch origin
git switch development
git pull --ff-only origin development
```

가상환경을 활성화합니다.

```bash
source ~/venv/mycobot/bin/activate
cd ~/Pinkk-robot-arm
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

## OpenCV 사전 확인

Hand-Eye와 ChArUco는 `opencv-contrib-python` 기능이 필요합니다.

```bash
python3 -c "
import cv2
import numpy
import scipy

print('OpenCV:', cv2.__version__)
print('NumPy:', numpy.__version__)
print('SciPy:', scipy.__version__)
print('aruco:', hasattr(cv2, 'aruco'))
print('calibrateHandEye:', hasattr(cv2, 'calibrateHandEye'))
"
```

`aruco` 또는 `calibrateHandEye`가 `False`이면 OpenCV 설치를 다시 확인합니다.
패키지를 교체하기 전에는 현재 상태를 백업합니다.

```bash
python3 -m pip freeze > ~/mycobot_packages_before_handeye.txt
```

## 카메라 intrinsic 위치

로봇 PC에는 intrinsic 결과 파일이 다음 위치에 있어야 합니다.

```text
src/robot_arm/robot_camera/camera_calibration/results/intrinsics.npz
```

`settings.py`의 기본 경로도 이 위치를 바라봅니다.

## serial port 확인

연결된 serial 장치를 확인합니다.

```bash
python3 -m serial.tools.list_ports -v
ls -l /dev/ttyUSB*
ls -l /dev/ttyACM*
```

다른 프로그램이 port를 점유 중인지 확인합니다.

```bash
sudo fuser -v /dev/ttyUSB0
lsof /dev/ttyUSB0
```

권한도 확인합니다.

```bash
ls -l /dev/ttyUSB0
groups
```

장치 group이 `dialout`이고 현재 사용자 group에 `dialout`이 없다면 추가합니다.

```bash
sudo usermod -aG dialout jetcobot
```

적용하려면 로그아웃 후 다시 로그인해야 합니다.

## 환경변수로 port 변경

기본값은 `/dev/ttyUSB0`, `1000000` baud입니다. 다른 값을 쓰는 환경에서는 소스 코드를
수정하지 않고 환경변수로 바꿉니다.

```bash
export JETCOBOT_PORT=/dev/실제_장치
export JETCOBOT_BAUD=1000000
```

현재 shell에 적용됐는지 확인합니다.

```bash
echo "$JETCOBOT_PORT"
echo "$JETCOBOT_BAUD"
```
