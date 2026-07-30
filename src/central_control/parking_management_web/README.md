# ROS 2 주차 관제 웹

이 화면은 Flask 백엔드 없이 정적 HTML로 실행합니다.

- 영상: `web_video_server` (`/pinkk/localization/image`, `/pinkk/lidar_map/image`)
- PINKY_01 배터리: rosbridge (`/pinky1/battery/percent`, `/pinky1/battery/voltage`)
- PINKY_02 배터리: rosbridge (`/pinky2/battery/percent`, `/pinky2/battery/voltage`)
  충전 중 순간 전압 변동이 퍼센트 표시에 그대로 반영되지 않도록 최근 7개
  표본의 중앙값과 완만한 지수 필터, 표시 deadband를 적용합니다.
- PINKY_01 제어 명령: rosbridge (`/pinky1/web/control`)
- PINKY_02 제어 명령: rosbridge (`/pinky2/web/control`)

PINKY_01의 긴급 정지 버튼은 상태 기록용 `/pinky1/web/control` 명령과 함께
`/pinky1/cmd_vel`에 속도가 모두 0인 `geometry_msgs/msg/Twist`를 3초 동안
20Hz로 발행합니다.

## 한 번에 실행

기존 관제 프로세스를 모두 종료한 뒤 프로젝트 루트에서 실행합니다.

```bash
cd /home/kukjiho/Pinkk
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
