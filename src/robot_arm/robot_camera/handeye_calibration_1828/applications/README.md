# USB 포트 수동 클릭 TF

이 단계에서는 로봇을 직접 제어하지 않습니다. 로봇 PC의 Flask MJPEG 영상을
노트북 OpenCV 창으로 받아 USB-A 포트의 네 모서리를 클릭하고, `solvePnP`로
다음 TF만 발행합니다.

```text
camera_optical_frame -> usb_port
```

이미 실행 중인 TF가 다음 행렬 곱을 처리하므로 최종 USB base 좌표는
`tf2_echo`로 확인합니다.

```text
T_base_usb
= T_base_flange @ T_flange_camera @ T_camera_usb
```

## 실행

로봇 PC에서는 `trajectory_bridge`와 Flask 카메라 서버를 실행합니다.
`charuco_tf_bridge`는 같은 카메라를 사용하므로 종료합니다. 노트북에서는
`real_execution.launch.py`와 실제 Hand-eye static TF를 실행합니다.

노트북에서:

```bash
source /opt/ros/jazzy/setup.bash
source ~/mycobot_moveit_ws/install/setup.bash

export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0

python3 -m \
  src.robot_arm.robot_camera.handeye_calibration_1828.applications.manual_usb_tf \
  --url http://192.168.6.1:5000/stream
```

OpenCV 창에서 `f`로 화면을 고정한 뒤 USB 외곽을 둘레 순서로 클릭합니다.
화면의 좌상단/우상단이 아니라 USB의 실제 긴 변과 짧은 변을 기준으로 해야 합니다.

```text
1번: 긴 변의 시작점
2번: 같은 긴 변의 끝점      (1→2 = 11.5 mm)
3번: 인접한 짧은 변의 끝점 (2→3 = 4.5 mm)
4번: 나머지 점
```

USB 긴 방향이 영상에서 세로라면 예를 들어 `좌상단 → 좌하단 → 우하단 → 우상단`
순서가 됩니다. 코드가 1→2 픽셀 길이보다 2→3 픽셀 길이가 더 긴 잘못된 클릭을
자동으로 거부합니다.

별도 노트북 터미널에서:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=36
export ROS_LOCALHOST_ONLY=0
ros2 run tf2_ros tf2_echo g_base usb_port
```

로봇을 움직인 뒤에는 `r`로 이전 클릭을 지우고 완전히 정지한 새 영상에서 다시
네 점을 클릭합니다.
