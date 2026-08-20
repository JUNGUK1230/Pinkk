# Robot Arm calibration assets

이 디렉터리는 로봇팔 카메라 내부보정과 Hand-eye 캘리브레이션 코드·결과를
관리합니다. 실제 ROS 2 인식과 제어 코드는 저장소 루트의 `ros2_ws/src`에서
관리합니다.

- `calibration/camera_intrinsics`: 카메라 내부 파라미터 보정
- `calibration/handeye`: Hand-eye 수집·계산·검증

구현이 없는 `motion_control`과 빈 공통 설정 자리표시자는 제거했습니다. 현재
운용 설정은 `ros2_ws/src/pinkk_usb_insertion/config`이 단일 기준입니다.
