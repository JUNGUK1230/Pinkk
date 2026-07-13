# 상단 카메라–LiDAR 정합 시스템 실행 가이드

## Camera BEV 한 프레임 저장

`capture_camera_bev.py`는 USB 카메라의 `1920×1080` 프레임에 왜곡 보정과 homography를 적용해 `1600×800` BEV를 표시합니다. BEV 창에서 `s`를 누르면 `camera_bev.png`로 저장하고, `q` 또는 `ESC`로 종료합니다. 기본 카메라는 스크립트 상단의 `CAMERA_ID = 2`입니다.

```bash
cd ~/PINKK/src/central_control/camera_tools/first_map
python3 capture_camera_bev.py
```

이 문서는 새 Ubuntu/Linux 환경에서 다음 기능을 실행하기 위한 설치 및 실행 절차입니다.

```text
USB Camera 1920×1080
    ↓
Camera Calibration 적용
    ↓
Lens Undistortion
    ↓
BEV 변환
    ↓
Camera → LiDAR Rigid Registration
    ↓
LiDAR Map 좌표계 정합
    ↓
ROS /map 좌표 변환
```

---

## 1. 압축파일을 풀 위치

가장 권장하는 위치는 사용자 홈 디렉터리 아래의 `project` 폴더입니다.

```text
~/project
```

예를 들어 새 PC의 사용자 이름이 `teamuser`라면 실제 경로는:

```text
/home/teamuser/project
```

기존 개발 PC의 사용자 이름이 `junguk`라면:

```text
/home/junguk/project
```

입니다.

### 압축파일이 `~/Downloads`에 있는 경우

예를 들어 압축파일 이름이:

```text
overhead_mapping.zip
```

이라면 아래처럼 실행합니다.

```bash
mkdir -p ~/project
unzip ~/Downloads/overhead_mapping.zip -d ~/project
```

압축파일 내부에 이미 `project/` 폴더가 들어 있다면 중첩 폴더가 생길 수 있습니다.

예:

```text
~/project/project/...
```

이 경우에는 압축파일 구조를 먼저 확인합니다.

```bash
unzip -l ~/Downloads/overhead_mapping.zip
```

압축파일 안쪽 최상단이 `project/`라면 홈 디렉터리에 바로 풉니다.

```bash
unzip ~/Downloads/overhead_mapping.zip -d ~/
```

최종적으로 반드시 아래와 같은 구조가 되도록 맞춥니다.

```text
~/project/
├── camera_calibration.npz
├── bev_homography.npz
├── camera_to_lidar_rigid_registration.npz
├── my_test_map0710.png
├── my_test_map0710.yaml
├── coordinate_transformer.py
├── live_camera_lidar_registration.py
├── live_coordinate_test.py
└── register_bev_lidar_rigid.py
```

---

## 2. 가장 중요한 경로 설정

기존 Python 코드 일부에는 다음 경로가 들어 있을 수 있습니다.

```python
PROJECT_DIR = Path(
    "/home/junguk/project"
)
```

새 PC의 사용자 이름이 `junguk`가 아니라면 반드시 수정해야 합니다.

### 권장 수정 방식

모든 스크립트에서 아래처럼 바꾸는 것을 권장합니다.

```python
from pathlib import Path

PROJECT_DIR = Path.home() / "project"
```

이렇게 하면 사용자 이름이 달라도 자동으로:

```text
/home/<현재사용자>/project
```

를 사용합니다.

예:

```text
/home/teamuser/project
/home/ubuntu/project
/home/pinky/project
```

---

## 3. 필수 파일

### 실행용 Python 파일

```text
coordinate_transformer.py
live_camera_lidar_registration.py
```

### 보정 및 정합 데이터

```text
camera_calibration.npz
bev_homography.npz
camera_to_lidar_rigid_registration.npz
```

### LiDAR Map

```text
my_test_map0710.png
my_test_map0710.yaml
```

### 검증 및 재정합용 파일

```text
live_coordinate_test.py
register_bev_lidar_rigid.py
```

---

## 4. Python 가상환경 생성

터미널에서:

```bash
cd ~/project
```

가상환경 생성:

```bash
python3 -m venv .venv
```

활성화:

```bash
source .venv/bin/activate
```

정상적으로 활성화되면 터미널 앞에 다음과 비슷하게 표시됩니다.

```text
(.venv) user@pc:~/project$
```

---

## 5. 필수 패키지 설치

먼저 pip를 업데이트합니다.

```bash
python -m pip install --upgrade pip
```

필수 패키지 설치:

```bash
pip install numpy opencv-python pyyaml
```

현재 단계의 카메라–LiDAR 정합 및 좌표 변환에는 최소한 위 패키지가 필요합니다.

설치 확인:

```bash
python -c "import cv2, numpy, yaml; print('OK')"
```

정상 출력:

```text
OK
```

---

## 6. USB 카메라 연결 확인

카메라 장치 확인:

```bash
ls -l /dev/video*
```

예:

```text
/dev/video0
/dev/video1
/dev/video2
```

현재 코드 기본값은:

```python
CAMERA_ID = 2
```

입니다.

새 PC에서 카메라 번호가 달라지면 각 실행 파일의:

```python
CAMERA_ID = 2
```

를 실제 장치 번호로 바꿉니다.

예:

```python
CAMERA_ID = 0
```

또는:

```python
CAMERA_ID = 3
```

### 카메라 상세 확인

`v4l2-ctl`이 없다면 설치:

```bash
sudo apt update
sudo apt install -y v4l-utils
```

장치 목록:

```bash
v4l2-ctl --list-devices
```

지원 포맷 확인:

```bash
v4l2-ctl -d /dev/video2 --list-formats-ext
```

현재 시스템은 1920×1080 MJPG 30 FPS를 사용합니다.

---

## 7. LiDAR YAML 확인

`my_test_map0710.yaml`의 `image:` 항목이 실제 PNG 파일명을 가리켜야 합니다.

권장 내용:

```yaml
image: my_test_map0710.png
mode: trinary
resolution: 0.010
origin: [-0.865, -1.539, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

확인:

```bash
cat ~/project/my_test_map0710.yaml
```

PNG 파일 존재 확인:

```bash
ls -l ~/project/my_test_map0710.png
```

---

## 8. 저장된 정합 결과 확인

정합 파일 존재 확인:

```bash
ls -l ~/project/camera_to_lidar_rigid_registration.npz
```

현재 저장된 정합 결과는 대략 다음 조건에서 생성되었습니다.

```text
BEV Size          : 1600 × 800 px
LiDAR Map Size    : 248 × 218 px
LiDAR Resolution  : 0.01 m/px
Origin            : [-0.865, -1.539, 0]
Registration RMSE : 약 1.68 cm
Rotation          : 약 39.50 deg
```

중요:

> 카메라 위치, 높이, 각도, 렌즈 설정, 맵 구조가 바뀌면 기존 정합 파일을 그대로 사용하면 안 됩니다.

---

## 9. 가장 먼저 실행할 것: 실시간 Camera–LiDAR Overlay

가상환경 활성화:

```bash
cd ~/project
source .venv/bin/activate
```

실행:

```bash
python live_camera_lidar_registration.py
```

정상 실행 시 다음 창들이 표시됩니다.

```text
LiDAR Map
Registered Camera
Live Camera-LiDAR Overlay
```

종료:

```text
Q
```

### 기대 결과

실시간 카메라 영상이:

```text
1920×1080 Raw Camera
    ↓
Undistortion
    ↓
1600×800 BEV
    ↓
Rigid Registration
    ↓
248×218 LiDAR Map Frame
```

순서로 처리되어 LiDAR 맵 위에 정렬됩니다.

---

## 10. 좌표 변환 테스트

실행:

```bash
python live_coordinate_test.py
```

BEV 화면의 임의 지점을 마우스로 클릭하면 터미널에 다음 좌표가 출력됩니다.

```text
BEV Pixel
LiDAR Pixel
ROS /map Coordinate [m]
```

예:

```text
BEV Pixel   : (800.00, 400.00)
LiDAR Pixel : (113.xx, 106.xx)
ROS /map    : (0.xxx, -0.xxx) m
```

종료:

```text
Q
```

---

## 11. 새 환경에서 정합을 다시 해야 하는 경우

다음 중 하나라도 바뀌면 재정합을 권장합니다.

```text
카메라 위치 변경
카메라 높이 변경
카메라 각도 변경
카메라 장착 방향 변경
렌즈/줌/초점 변경
BEV Homography 변경
LiDAR 맵 변경
주요 구조물 배치 변경
```

재정합 실행:

```bash
python register_bev_lidar_rigid.py
```

조작키:

```text
S : 현재 BEV 고정
U : 마지막 Camera 대응점 삭제
I : 마지막 LiDAR 대응점 삭제
R : 전체 초기화
W : 정합 계산 + 저장
Q : 종료
```

### 대응점 선택

Camera BEV와 LiDAR Map에서 동일한 실제 지점을 같은 순서로 클릭합니다.

예:

```text
Camera Point 1 ↔ LiDAR Point 1
Camera Point 2 ↔ LiDAR Point 2
Camera Point 3 ↔ LiDAR Point 3
...
```

가능하면:

```text
고정 구조물 모서리
벽 모서리
바닥 접점
LiDAR에서도 명확히 보이는 지점
```

을 사용합니다.

높이가 있는 구조물의 상단점은 바닥 평면 BEV와 차이가 날 수 있으므로 주의합니다.

### 저장

`W`를 누르면 다음 파일이 저장됩니다.

```text
~/project/camera_to_lidar_rigid_registration.npz
~/project/camera_lidar_rigid_overlay.png
```

---

## 12. 실행 순서 요약

새 PC에서는 아래 순서대로 하면 됩니다.

### 1단계: 압축 해제

```bash
mkdir -p ~/project
unzip ~/Downloads/overhead_mapping.zip -d ~/project
```

### 2단계: 폴더 이동

```bash
cd ~/project
```

### 3단계: 가상환경 생성

```bash
python3 -m venv .venv
```

### 4단계: 가상환경 활성화

```bash
source .venv/bin/activate
```

### 5단계: 라이브러리 설치

```bash
pip install --upgrade pip
pip install numpy opencv-python pyyaml
```

### 6단계: 카메라 확인

```bash
ls -l /dev/video*
```

### 7단계: 코드의 `CAMERA_ID` 확인

```python
CAMERA_ID = 2
```

### 8단계: 코드의 프로젝트 경로 확인

권장:

```python
PROJECT_DIR = Path.home() / "project"
```

### 9단계: 실시간 정합 실행

```bash
python live_camera_lidar_registration.py
```

### 10단계: 좌표 변환 확인

```bash
python live_coordinate_test.py
```

---

## 13. 자주 발생하는 오류

### 오류 1: 파일 없음

예:

```text
FileNotFoundError
```

확인:

```bash
ls -al ~/project
```

특히 아래 파일이 모두 있어야 합니다.

```text
camera_calibration.npz
bev_homography.npz
camera_to_lidar_rigid_registration.npz
my_test_map0710.png
my_test_map0710.yaml
```

---

### 오류 2: 카메라 열기 실패

예:

```text
카메라 2번 열기 실패
```

장치 확인:

```bash
ls -l /dev/video*
```

그다음 코드의:

```python
CAMERA_ID = 2
```

를 수정합니다.

---

### 오류 3: 1920×1080으로 열리지 않음

카메라 포맷 확인:

```bash
v4l2-ctl -d /dev/video2 --list-formats-ext
```

코드가 MJPG를 설정하는지 확인:

```python
fourcc = cv2.VideoWriter_fourcc(*"MJPG")
cap.set(cv2.CAP_PROP_FOURCC, fourcc)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FPS, 30)
```

---

### 오류 4: YAML이 잘못된 이미지 파일을 가리킴

확인:

```bash
cat ~/project/my_test_map0710.yaml
```

첫 줄이 실제 파일명과 일치해야 합니다.

```yaml
image: my_test_map0710.png
```

---

### 오류 5: Qt 폰트 경고

예:

```text
QFontDatabase: Cannot find font directory ...
```

정합 계산이나 파일 저장이 정상이라면 대부분 치명적 오류가 아닙니다.

---

## 14. 권장 최종 폴더 구조

```text
~/project/
├── .venv/
│
├── camera_calibration.npz
├── bev_homography.npz
├── camera_to_lidar_rigid_registration.npz
│
├── my_test_map0710.png
├── my_test_map0710.yaml
│
├── coordinate_transformer.py
├── live_camera_lidar_registration.py
├── live_coordinate_test.py
├── register_bev_lidar_rigid.py
│
├── camera_lidar_rigid_overlay.png
└── README.md
```

---

## 15. 팀원에게 전달할 핵심 주의사항

1. 압축 해제 위치는 `~/project` 권장
2. 새 PC에서는 `CAMERA_ID`가 달라질 수 있음
3. 코드의 `/home/junguk/project` 하드코딩은 새 사용자에서 수정 필요
4. 가장 권장되는 프로젝트 경로 코드는 다음과 같음

```python
PROJECT_DIR = Path.home() / "project"
```

5. 카메라 설치 위치가 바뀌면 기존 정합 파일을 그대로 사용하지 말 것
6. LiDAR 맵이 바뀌면 rigid registration을 다시 수행할 것
7. 현재 좌표계 최종 기준은 ROS `/map`
8. 현재 LiDAR 맵 resolution은 `0.01 m/px`
9. 현재 저장된 정합 RMSE는 약 `1.68 cm`
