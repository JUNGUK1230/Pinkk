# ROS 2 주차 관제 웹

이 화면은 Flask 백엔드 없이 정적 HTML로 실행합니다.

- 영상: `web_video_server` (`/pinkk/localization/image`, `/pinkk/lidar_map/image`)
- PINKY_01 배터리: rosbridge (`/pinky1/battery/percent`, `/pinky1/battery/voltage`)
- PINKY_02 배터리: rosbridge (`/pinky2/battery/percent`, `/pinky2/battery/voltage`)
- 제어 명령: rosbridge (`/pinkk/web/control`)

```bash
sudo apt install ros-$ROS_DISTRO-web-video-server ros-$ROS_DISTRO-rosbridge-server
ros2 run web_video_server web_video_server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
python3 -m http.server 8000 --directory src/central_control/parking_management_web
```

브라우저에서 `http://ROS_PC_IP:8000`으로 접속합니다. 다른 토픽이나 포트를
사용하면 URL 쿼리로 지정할 수 있습니다.

```text
http://ROS_PC_IP:8000/?videoPort=8080&bridgePort=9090&yoloTopic=/pinkk/localization/image&lidarTopic=/pinkk/lidar_map/image
```

차량 ROS 토픽은 로봇별 네임스페이스를 사용합니다.

```text
/pinky1/cmd_vel             /pinky2/cmd_vel
/pinky1/odom                /pinky2/odom
/pinky1/scan                /pinky2/scan
/pinky1/joint_states        /pinky2/joint_states
/pinky1/battery/percent     /pinky2/battery/percent
/pinky1/battery/voltage     /pinky2/battery/voltage
```
