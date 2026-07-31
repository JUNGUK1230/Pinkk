# ROS 2 주차 관제 웹

이 화면은 Flask 백엔드 없이 정적 HTML로 실행합니다.

- 영상: `web_video_server` (`/pinkk/localization/image`, `/pinkk/lidar_map/image`)
- 주차장·경로 상태: rosbridge (`/pinkk/management/status`)
- 분야별 알림 이벤트: rosbridge (`/pinkk/management/events`, 연결 예정)
- PINKY_01 배터리: rosbridge (`/pinky1/battery/percent`, `/pinky1/battery/voltage`)
- PINKY_02 배터리: rosbridge (`/pinky2/battery/percent`, `/pinky2/battery/voltage`)
  충전 중 순간 전압 변동이 퍼센트 표시에 그대로 반영되지 않도록 최근 7개
  표본의 중앙값과 완만한 지수 필터, 표시 deadband를 적용합니다.
- 관리자 제어 요청: rosbridge (`/pinkk/web/control`)

관리자 웹은 Pinky Pro의 `/cmd_vel`을 직접 발행하지 않습니다. 모든 버튼은
대상 로봇, 명령, 우선순위, 요청 시각을 JSON 문자열로 만들어 중앙제어의
`/pinkk/web/control`에만 발행합니다. 중앙제어 명령 중재기는 요청을 판정한
뒤 Pinky Pro에 최종 명령을 한 경로로 전달해야 합니다.

명령 우선순위는 숫자가 작을수록 높으며 `emergency_stop(1)`,
`pause(2)`, `replan(3)`, `exit(4)`, `charge(5)`, `entry(6)` 순서입니다.
긴급정지는 해제 명령 전까지 유지되는 래치 상태로 처리하고, 이 상태에서는
일반 명령을 거부해야 합니다. 현재 저장소에는 이 중앙 명령 중재 노드와
Pinky Pro 측 최종 명령 수신기가 아직 구현되어 있지 않습니다.

주차장 요약과 경로 생성 현황은 HTML에 고정하지 않습니다.
`live_localization.py`가 YOLO 주차면 점유 결과와 통합 Hybrid A* 상태를
`/pinkk/management/status`에 JSON 문자열로 발행하고, 웹이 이를 구독해
전체 일반 주차면, 사용 중, 점유율, 충전 가능 수, 목적지, 경로 상태와
예상 완료 시간을 갱신합니다. 예상 완료 시간은 검증된 trajectory의 구간
거리와 목표 속도로 계산한 근삿값이며 실제 주행 지연은 포함하지 않습니다.

공간 판정도 중앙제어에서 수행합니다. P1~P10과 C1~C2는 YOLO segmentation
차량 mask와 고정 공간 polygon의 겹침 비율로 점유를 판단합니다. 입구와
출구는 `parking_points.json`의 네 BEV 모서리로 polygon을 만들고, 현재
ByteTrack 검출 차량의 중심점이 그 안에 있는지 판정합니다. 웹은 영상
픽셀을 다시 분석하지 않고 `/pinkk/management/status`의 `spaces` 결과만
표시합니다.

## 분야별 알림 이벤트

웹은 긴급·시스템, 경로 생성, 입·출차, 충전, 주차 공간의 다섯 창에
지원할 알림 종류를 모두 표시합니다. 중앙제어 연결 전에는 각 창을
`설계 항목`으로 표시하며, 실제 이벤트를 받으면 해당 행을 발생 시각과
함께 강조합니다. 왼쪽 원 색상과 등급 문구를 함께 사용합니다.

- 빨강 `긴급`: 즉시 확인
- 주황 `오류`: 실패·연결 장애
- 노랑 `주의`: 점유·부족·재시도
- 파랑 `정보`: 요청·감지·시작
- 초록 `정상`: 완료·연결·사용 가능
- 회색 `대기`: 처리 입력 대기

향후 중앙제어 이벤트 토픽은 `/pinkk/management/events`이며 메시지 타입은
`std_msgs/msg/String`입니다. `data`에는 다음 JSON을 넣습니다.

```json
{
  "category": "route",
  "event": "route_generation_failed",
  "severity": "error",
  "robot_id": "pinky1",
  "message": "P8 경로 생성 실패",
  "occurred_at": "2026-07-31T14:30:00+09:00"
}
```

`category`는 `system`, `route`, `access`, `charging`, `parking` 중 하나를
사용합니다. 중앙제어가 아직 이 토픽을 발행하지 않으므로 현재 실제 강조가
가능한 항목은 웹 버튼 요청, rosbridge 연결·끊김, 중앙 상태 토픽
수신 시작·중단입니다.

## 한 번에 실행

기존 관제 프로세스를 모두 종료한 뒤 프로젝트 루트에서 실행합니다.

```bash
cd /home/junguk/PINKK
./src/central_control/scripts/run_parking_management.sh
```

핑키 SSH bringup까지 함께 시작하려면 `ssh-copy-id pinky@192.168.0.99`로
SSH 키 인증을 먼저 설정하고 다음과 같이 실행합니다.

```bash
./src/central_control/scripts/run_parking_management.sh --with-pinky
```

영상 서버, rosbridge, `index.html` 서버와 localization이 함께 실행됩니다.
localization은 YOLO 화면과 빨간 차량 좌표가 표시된 실제 LiDAR 맵을 같은
프레임에서 각각 `/pinkk/localization/image`, `/pinkk/lidar_map/image`로
발행합니다.
종료할 때는 같은 터미널에서 `Ctrl+C`를 누릅니다. 로그는
`.runtime/parking_management/`에 저장됩니다.

## 개별 실행

```bash
sudo apt install ros-$ROS_DISTRO-web-video-server ros-$ROS_DISTRO-rosbridge-server
ros2 run web_video_server web_video_server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
python3 -m http.server 8000 --directory src/central_control/parking_management_web
```

## 핑키 bringup

위 서버들을 실행한 다음 핑키에 SSH로 접속해 bringup을 실행합니다. ROS PC와
핑키의 `ROS_DOMAIN_ID`는 반드시 같아야 합니다.

```bash
ssh pinky@192.168.0.99
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
export ROS_DOMAIN_ID=36
ros2 launch pinky_bringup bringup_robot.launch.xml namespace:=pinky1
```

bringup 터미널은 종료하지 않고 유지합니다. 새 핑키 터미널에서 배터리
데이터가 발행되는지 확인합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/pinky_pro/install/setup.bash
export ROS_DOMAIN_ID=36
ros2 topic list | grep battery
ros2 topic echo /pinky1/battery/percent --once
```

PINKY_01 bringup의 배터리 토픽은 `/pinky1/battery/percent`와
`/pinky1/battery/voltage`를 사용합니다.

## 접속 주소

같은 ROS PC의 브라우저에서는 아래 주소로 접속합니다.

```text
http://localhost:8000
```

다른 PC나 휴대폰에서는 ROS PC와 같은 네트워크에 연결한 뒤 ROS PC에서
`hostname -I`를 실행해 IP를 확인합니다. 예를 들어 IP가
`192.168.0.73`이면 아래 주소로 접속합니다. `ROS_PC_IP`라는 문자열을
주소창에 그대로 입력하지 않습니다.

```text
http://192.168.0.73:8000
```

다른 토픽이나 포트를 사용하면 URL 쿼리로 지정할 수 있습니다.

```text
http://192.168.0.73:8000/?videoPort=8080&bridgePort=9090&yoloTopic=/pinkk/localization/image&lidarTopic=/pinkk/lidar_map/image
```

차량 ROS 토픽은 PINKY_01 네임스페이스만 사용합니다.

```text
/pinky1/cmd_vel
/pinky1/odom
/pinky1/scan
/pinky1/joint_states
/pinky1/battery/percent
/pinky1/battery/voltage
```
