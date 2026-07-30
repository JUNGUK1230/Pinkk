# Smart Parking System

상단 카메라 기반 주차 감지·BEV/LiDAR 지도 정합·경로 계획과 ROS 2 차량 제어를 위한 프로젝트입니다. 프로젝트는 단일 ROS 2 Python 패키지 `pinkk`로 구성되어 있습니다.

## 구성

- `src/central_control`: 상단 카메라, 보정·BEV, YOLO 주차 감지, 지도 정합 및 경로 계획
- `src/vehicle_control`: `/odom`을 받아 `/cmd_vel`을 발행하는 PID 경로 추종 노드
- `src/robot_arm`: 로봇팔 카메라·동작 제어 관련 코드와 설정

## Python 의존성 설치

카메라·YOLO·경로 계획 도구를 실행할 때 필요합니다.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## ROS 2 빌드 및 실행

ROS 2 Jazzy 환경에서 프로젝트 루트에서 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## PINKY_01 브링업

PINKY_01의 주소는 `192.168.0.99`, 계정은 `pinky`, ROS 도메인은 `36`을
기준으로 합니다. 중앙 관제 PC와 PINKY_01은 같은 네트워크에 연결되어 있어야
합니다.

### SSH 키 등록

아래 명령은 PINKY_01 터미널이 아니라 **PINKY_01에 접속할 중앙 관제 PC
터미널**에서 한 번만 실행합니다.

```bash
ssh-keygen -t ed25519
ssh-copy-id pinky@192.168.0.99
ssh pinky@192.168.0.99
```

`ssh-keygen` 실행 중 저장 위치와 암호를 물으면 별도 설정이 없는 경우 Enter를
눌러 기본값을 사용합니다.

### PINKY_01에서 직접 실행

중앙 관제 PC에서 PINKY_01에 접속한 다음 bringup을 실행합니다.

```bash
ssh pinky@192.168.0.99
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
export ROS_DOMAIN_ID=36
ros2 launch pinky_bringup bringup_robot.launch.xml namespace:=pinky1
```

bringup을 종료할 때는 해당 터미널에서 `Ctrl+C`를 누릅니다.

### 중앙 관제 PC에서 한 줄로 실행

SSH 키 등록이 끝났다면 중앙 관제 PC에서 다음 명령으로 원격 bringup을 바로
실행할 수 있습니다.

```bash
ssh -t pinky@192.168.0.99 \
  'source /opt/ros/jazzy/setup.bash && source ~/pinky_pro/install/setup.bash && export ROS_DOMAIN_ID=36 && ros2 launch pinky_bringup bringup_robot.launch.xml namespace:=pinky1'
```

### 토픽 확인

bringup을 유지한 상태에서 새 터미널로 PINKY_01에 접속합니다.

```bash
ssh pinky@192.168.0.99
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
export ROS_DOMAIN_ID=36
```

먼저 노드와 전체 토픽을 확인합니다.

```bash
ros2 node list
ros2 topic list -t
```

어제 수정한 네임스페이스가 제대로 적용됐는지는 아래 코드로 한 번에
검사합니다. 모든 필수 토픽이 있고 예전 루트 토픽이 없으면 마지막에
`토픽 네임스페이스 검사: 정상`이 출력됩니다.

```bash
expected_topics=(
  /pinky1/cmd_vel
  /pinky1/odom
  /pinky1/scan
  /pinky1/joint_states
  /pinky1/robot_description
  /pinky1/battery/percent
  /pinky1/battery/voltage
)

legacy_topics_regex='^/(cmd_vel|odom|scan|joint_states|robot_description|battery/percent|battery/voltage)$'
topic_list="$(ros2 topic list)"
failed=0

for topic in "${expected_topics[@]}"; do
  if grep -Fqx "$topic" <<<"$topic_list"; then
    echo "[OK] $topic"
  else
    echo "[누락] $topic"
    failed=1
  fi
done

legacy_topics="$(grep -E "$legacy_topics_regex" <<<"$topic_list" || true)"
if [[ -n "$legacy_topics" ]]; then
  echo "[오류] 네임스페이스 없는 예전 토픽이 남아 있습니다:"
  echo "$legacy_topics"
  failed=1
else
  echo "[OK] 예전 루트 토픽 없음"
fi

if (( failed )); then
  echo "토픽 네임스페이스 검사: 실패"
else
  echo "토픽 네임스페이스 검사: 정상"
fi
```

publisher와 subscriber 연결 상태를 확인합니다.

```bash
ros2 topic info -v /pinky1/battery/percent
ros2 topic info -v /pinky1/battery/voltage
ros2 topic info -v /pinky1/cmd_vel
ros2 topic info -v /pinky1/odom
ros2 topic info -v /pinky1/scan
```

마지막으로 실제 메시지가 들어오는지 확인합니다. 각 명령은 최대 10초 뒤
자동으로 종료됩니다.

```bash
timeout 10 ros2 topic echo /pinky1/battery/percent --once
timeout 10 ros2 topic echo /pinky1/battery/voltage --once
timeout 10 ros2 topic echo /pinky1/odom --once
timeout 10 ros2 topic echo /pinky1/scan --once
```

정상 상태에서는 차량 토픽이 `/pinky1` 네임스페이스 아래에 있어야 합니다.

```text
/pinky1/cmd_vel
/pinky1/odom
/pinky1/scan
/pinky1/joint_states
/pinky1/battery/percent
/pinky1/battery/voltage
```

### 차량 PID 경로 추종

`pid_path_follower`는 기본적으로 `/odom`을 구독하고 `/cmd_vel`을 발행합니다. 웨이포인트와 제어 이득은 현재 노드 코드에 정의돼 있습니다.

```bash
ros2 run pinkk pid_path_follower
```

토픽을 바꾸려면 ROS 파라미터를 전달합니다.

```bash
ros2 run pinkk pid_path_follower --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel
```

### 중앙 제어 진입점

```bash
ros2 run pinkk central_control
```

현재 중앙 제어 파이프라인은 모듈을 연결하기 위한 골격으로, 실행 시 안내 메시지만 출력합니다.

## 개발 도구 실행

프로젝트 루트에서 실행합니다.

상단 카메라 미리보기(기본 장치 번호·해상도는 `src/central_control/config/camera/camera.yaml`에서 설정):

```bash
python3 -m src.central_control.overhead_vision.camera.camera_capture
```

YOLO 세그멘테이션·BEV·LiDAR 지도 표시:

```bash
./src/central_control/scripts/run_yolo_seg.sh
```

이 스크립트는 `.venv`와 `/dev/video2`를 확인합니다. YOLO 가중치 파일은 `src/central_control/models/best.pt`에 두어야 하며, 모델 파일은 Git에서 제외됩니다.

### 상단 카메라 실시간 차량 위치·주차면 좌표

새 localization 파이프라인은 USB 카메라 원본을 왜곡 보정한 뒤 `1600×800` BEV로 변환하고, `best.pt` 차량 segmentation mask를 Camera BEV에서 LiDAR map 좌표로 변환합니다. 차량 위치는 Hybrid A*와 같은 **rear axle 중심 cm pose**로 출력하며, 고정 주차면은 차량 mask와 겹친 비율로 점유 여부를 판단합니다.

```bash
cd ~/PINKK
.venv/bin/python -m src.central_control.overhead_vision.localization.live_localization
```

처리된 annotation 화면은 `/pinkk/localization/image` (`sensor_msgs/Image`,
`bgr8`)로 함께 발행됩니다. Flask 없이 웹에서 보려면 ROS 2
`web_video_server`를 실행한 뒤 정적 페이지를 여십시오.

### 관제 시스템 한 번에 실행

기존 관제 프로세스를 모두 종료한 뒤 프로젝트 루트에서 실행합니다. 영상
서버, rosbridge, `index.html` 정적 서버와 localization을 한꺼번에 시작하며,
localization은 YOLO 화면과 빨간 차량 좌표가 표시된 실제 LiDAR 맵을 같은
프레임에서 함께 발행합니다.
브라우저도 자동으로 엽니다.

```bash
cd /home/kukjiho/Pinkk
./src/central_control/scripts/run_parking_management.sh
```

SSH로 핑키 bringup까지 함께 시작하려면 다음 옵션을 사용합니다. 백그라운드
실행을 위해 `ssh-copy-id pinky@192.168.0.99`로 SSH 키 인증을 먼저 설정해야
하며, `PINKY_HOST` 환경 변수로 접속 주소를 바꿀 수 있습니다.

```bash
./src/central_control/scripts/run_parking_management.sh --with-pinky
```

스크립트를 실행한 터미널에서 `Ctrl+C`를 누르면 스크립트가 시작한 프로세스가
함께 종료됩니다. 로그는 `.runtime/parking_management/`에 저장됩니다.

### 관제 시스템 개별 실행

```bash
sudo apt install ros-$ROS_DISTRO-web-video-server
ros2 run web_video_server web_video_server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
python3 -m http.server 8000 --directory src/central_control/parking_management_web
```

웹 서버와 함께 PINKY_01을 실행하려면 위의 [PINKY_01 브링업](#pinky_01-브링업)
명령을 별도 터미널에서 실행합니다. 중앙 관제 PC와 PINKY_01의
`ROS_DOMAIN_ID`는 반드시 `36`으로 같아야 합니다.

같은 ROS PC의 브라우저에서는 `http://localhost:8000`으로 접속합니다.
다른 PC나 휴대폰에서는 ROS PC와 같은 네트워크에 연결한 뒤
`hostname -I`로 ROS PC의 IP를 확인하고 `http://<ROS_PC_IP>:8000`으로
접속합니다. 예를 들어 IP가 `192.168.0.73`이면
`http://192.168.0.73:8000`입니다. 통합 관제 화면은
영상에 `web_video_server`, 배터리와 제어 명령에 rosbridge를 사용해
PINKY_01의 `/pinky1/battery/percent`, `/pinky1/battery/voltage`와
`/pinky1/web/control` 토픽을 직접 사용합니다. LiDAR
영상 토픽 기본값은 `/pinkk/lidar_map/image`이며 URL의 `lidarTopic` 쿼리로
바꿀 수 있습니다. `--no-display`로 로컬 OpenCV 창을 꺼도 ROS 영상 발행은
유지됩니다.

직접 파일 실행도 지원하지만 YOLO·OpenCV가 설치된 프로젝트 `.venv`의 module 실행을 권장합니다. 시스템 `python3`를 사용하면 환경에 따라 `ultralytics`가 없을 수 있습니다.

최신 결과는 `src/central_control/path_planning/output/live_vision_scene.json`에 매 프레임 원자적으로 저장됩니다. 다른 터미널에서 경로계획 입력을 확인할 수 있습니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/read_live_vision_scene.py
```

차량이 발견된 fresh scene으로 한 번 자동 경로를 생성하려면 localization 창에서 수동 heading을 지정한 뒤, localization을 계속 실행한 상태에서 다음 명령을 사용합니다. 현재 고정 목표 P5의 두 진입 heading 중 충돌 없는 12cm 후진주차 approach를 선택하고, approach에 전진 도착·정지한 뒤 직선 후진합니다. footprint 보정·곡률 smoothing·속도 프로파일·종단 후진 validator를 모두 통과한 경로만 저장합니다.

현재 LiDAR occupancy에서 P5 polygon 내부는 원본 obstacle 약 23.1%, 2cm inflation 후 약 53.3%로 측정되어 중앙의 12cm 후진 corridor가 차단됩니다. planner는 이를 탐색 전에 감지해 즉시 실패 처리합니다. P5 실주차 전에는 지도에서 주행 가능한 주차선·바닥이 obstacle로 포함됐는지 확인하고 map/BEV 정합을 보정해야 합니다.

실시간 계획은 전체 기본 30초, goal 후보당 기본 5초로 제한됩니다. 탐색 중에는 후보 번호와 확장 노드 수가 출력되며, 진단 실행에서만 `--planning-timeout-sec 0 --candidate-timeout-sec 0`으로 무제한을 지정합니다.

Hybrid 탐색은 차량 rectangle 바깥에 2cm obstacle inflation을 추가해, 폭 10cm 차량의 중심 경로가 측면 장애물에서 약 7cm 이상 떨어지도록 합니다. 폭 20cm 주차면 안에서 양쪽 안전공간을 유지하면서 smoothing 경로가 벽을 침범하지 않게 확보하는 여유입니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/plan_from_live_vision.py
```

생성 파일은 `output/live_hybrid_path_world_cm.csv`, `output/live_hybrid_path_camera_bev.csv`, `output/live_hybrid_path_world_cm.json` 및 `output/live_hybrid_path_on_camera_bev.png`입니다. 경로 이미지는 최신 `live_camera_bev.png` 위에 빨간 경로, 초록 start heading, 파란 goal heading을 표시합니다. 이번 단계는 한 번 계획하고 종료하는 기본 파이프라인이며 제어기로 전송하지 않습니다. 실패 시 이전 자동 경로와 overlay를 제거하고 `output/live_hybrid_planning_status.json`에 차단 이유를 기록합니다.

현재 차량 segmentation 장축만으로는 앞/뒤가 180° 모호하므로 기본 실행에서는 heading을 자동 확정하지 않습니다. localization 창에서 `h`를 누른 뒤 검출 차량의 **앞쪽 지점**을 클릭하면 해당 방향을 수동 heading으로 고정합니다. `x`를 누르면 지우고 다시 지정할 수 있습니다. 목표 주차면은 현재 `P5`로 고정되며 P5가 점유 상태면 계획을 차단합니다. 화면에 차량이 여러 대면 `initial_ego_center_bev_px` 또는 `--initial-ego-center`로 ego 후보도 지정할 수 있습니다.

별도 터미널에서 `plan_from_live_vision.py`를 실행해 검증 경로가 저장되면 localization 창이 파일을 자동으로 읽어 **굵은 빨간 연속선**으로 표시합니다. 이전 C1/P6 경로처럼 현재 P5 목표와 다른 결과는 표시하지 않습니다.

저장된 BEV 이미지로 카메라 없이 전체 변환을 확인할 수도 있습니다.

```bash
cd ~/PINKK
.venv/bin/python -m src.central_control.overhead_vision.localization.live_localization \
  --bev-image src/central_control/camera_tools/first_map/camera_bev.png \
  --initial-ego-center 67 585 \
  --initial-yaw-deg -117 \
  --no-display
```

`yolo11l.pt` 기본 가중치는 detection 모델이므로 mask 기반 헤딩·주차면 점유 계산에는 사용할 수 없습니다. 라벨링과 재학습이 끝나면 동일한 `car` class를 가진 YOLO11 Large **segmentation** 가중치로 `model_path`만 교체합니다.

기타 카메라 보정·BEV·지도 정합·경로 계획 도구의 사용법은 [중앙 제어 도구 문서](src/central_control/camera_tools/README.md), [LiDAR 지도 문서](src/central_control/map/lidar_map/README.md), [경로 계획 문서](src/central_control/path_planning/README.md)를 참고하세요.

## 참고

- ROS 2 패키지 메타데이터는 `src/package.xml`, 패키지 실행 명령은 `src/setup.py`에서 관리합니다.
- 카메라 장치 번호는 일반 카메라 미리보기 설정(`camera.yaml`)과 YOLO 실행 스크립트(`/dev/video2`)가 서로 다를 수 있으므로, 연결된 장치를 확인한 뒤 맞춰 사용하세요.
