# Path Planning 파일 구성

## 설정 파일

| 파일 | 역할 |
|---|---|
| `config/vehicle_config.yaml` | 차량 길이·폭·wheelbase·rear overhang·조향 한계를 정의합니다. |
| `config/planner_config.yaml` | Hybrid A*, Reeds–Shepp, smoothing, 속도 프로파일 및 validator 파라미터를 정의합니다. |

## 경로계획 모듈

| 파일 | 역할 |
|---|---|
| `src/occupancy_grid.py` | 지도 image와 YAML을 occupancy grid로 변환하고 외부 영역 차단·obstacle inflation을 수행합니다. |
| `src/hybrid_astar_planner.py` | 차량 yaw, 조향, 전진·후진, 회전 footprint를 고려해 경로를 탐색합니다. |
| `src/reeds_shepp.py` | 전진·후진이 가능한 Reeds–Shepp analytic goal 연결 후보를 생성합니다. |
| `src/t_parking_planner.py` | 통로 staging 접근과 최종 후진 maneuver를 결합한 T자 후면주차 경로를 만듭니다. |
| `src/path_smoothing.py` | 기어 전환을 유지하면서 경로 위치·yaw·곡률을 부드럽게 만듭니다. |
| `src/trajectory_profile.py` | 곡률에 따른 목표속도, 각속도 및 정지점을 계산합니다. |
| `src/trajectory_validator.py` | 충돌, 간격, yaw, 조향, 속도, 기어 전환 조건을 검사합니다. |
| `src/vision_scene_input.py` | 실시간 scene의 시간·좌표·planning-ready 상태를 검증해 planner 입력으로 변환합니다. |
| `src/visualization.py` | occupancy grid와 Camera BEV 위에 경로·start·goal을 그립니다. |
| `src/__init__.py` | path-planning Python API를 노출합니다. |

## 실행·진단 스크립트

| 파일 | 역할 |
|---|---|
| `scripts/plan_from_live_vision.py` | 최신 YOLO scene으로 T자 후진주차 경로를 한 번 생성하고 검증 결과를 저장합니다. |
| `scripts/read_live_vision_scene.py` | 현재 planner에 전달될 차량 pose와 주차면 goal을 출력합니다. |
| `scripts/test_live_vision_scene.py` | ego 선택, 수동 heading, 주차면 점유 및 stale scene 차단을 회귀 검사합니다. |
| `scripts/test_t_parking.py` | staging과 마지막 후진을 포함한 T자 주차 경로를 회귀 검사합니다. |
| `scripts/test_plan_from_live_vision.py` | vision scene부터 Hybrid A* 저장까지 통합 흐름을 회귀 검사합니다. |
| `scripts/test_trajectory_profile.py` | 곡률 기반 속도와 기어 전환 정지점 계산을 검사합니다. |
| `scripts/test_trajectory_validator.py` | 정상·비정상 trajectory의 안전 차단 조건을 검사합니다. |
| `scripts/test_path_publisher.py` | 경로·차량 pose ROS message 변환과 stale 차단을 검사합니다. |

## 실시간 출력 파일

| 파일 | 역할 |
|---|---|
| `output/live_vision_scene.json` | YOLO가 생성한 최신 차량 pose, 주차면 상태 및 planning request입니다. |
| `output/live_camera_bev.png` | scene과 같은 frame의 최신 Camera BEV입니다. |
| `output/live_hybrid_path_world_cm.csv` | 제어용 Hybrid trajectory를 표 형식으로 저장합니다. |
| `output/live_hybrid_path_camera_bev.csv` | 같은 trajectory의 Camera BEV pixel 좌표입니다. |
| `output/live_hybrid_path_world_cm.json` | publisher가 읽는 검증된 trajectory와 planner metadata입니다. |
| `output/live_hybrid_path_on_camera_bev.png` | 최신 Camera BEV 위에 경로와 start·goal을 표시한 이미지입니다. |
| `output/live_hybrid_planning_status.json` | 최근 계획 성공 여부와 실패 원인을 저장합니다. |

## 문서

| 파일 | 역할 |
|---|---|
| `DEVELOPMENT_MEMO.md` | 구현 변경, 판단 근거, 회귀 결과와 다음 작업을 기록합니다. |
| `THIRD_PARTY_NOTICES.md` | Reeds–Shepp 구현 등 외부 코드의 출처와 라이선스를 기록합니다. |
| `README.md` | 이 폴더의 파일 역할을 설명합니다. |
