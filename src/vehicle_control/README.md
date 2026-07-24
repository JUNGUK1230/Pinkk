# Vehicle Control

이 폴더는 중앙제어에서 발행한 경로를 차량의 속도 명령으로 변환하는
ROS 2 차량 제어 후보 코드를 보관한다.

- `pid_path_follower_smooth_topic.py`: `/pinkk/planned_path` (`nav_msgs/Path`)를
  받아 전진·후진을 추정하고 PID 제어로 `/cmd_vel`을 발행한다.
  중앙 경로를 사용하지 않을 때는 내장된 WP7 기준 주행·주차 시퀀스를
  사용한다.
- `pid_path_follower_smooth_parking_complete_topic.py`: 동일한
  `/pinkk/planned_path` 연동 제어기다. 내장 waypoint 모드에서 WP8
  주차 완료 지점까지 곡선으로 연결한 대안이다.
- `config/vehicle/vehicle.yaml`: 차량 제어에 사용할 차체 제원 설정이다.
- `__init__.py`: `vehicle_control` Python 패키지 표시 파일이다.

두 제어기는 동일한 ROS 2 노드 이름과 `/cmd_vel` 출력을 사용하므로
둘 중 하나만 실행해야 한다. 현재는 `/pinkk/planned_trajectory`의 명시적인
`direction`, 조향각, 목표 속도, 정지 표시는 사용하지 않는다.
