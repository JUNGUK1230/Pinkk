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
| `src/t_parking_planner.py` | 2D 통로 guide, 짧은 Hybrid 접근 구간과 최종 후진 maneuver를 결합한 T자 후면주차 경로를 만듭니다. |
| `src/path_smoothing.py` | 기어 전환을 유지하면서 경로 위치·yaw·곡률을 부드럽게 만듭니다. |
| `src/trajectory_profile.py` | 곡률에 따른 목표속도, 각속도 및 정지점을 계산합니다. |
| `src/trajectory_validator.py` | 충돌, 간격, yaw, 조향, 속도, 기어 전환 조건을 검사합니다. |
| `src/vision_scene_input.py` | 실시간 scene의 시간·좌표·planning-ready 상태를 검증해 planner 입력으로 변환합니다. |
| `src/visualization.py` | occupancy grid와 Camera BEV 위에 경로·start·goal을 그립니다. |
| `src/__init__.py` | path-planning Python API를 노출합니다. |

## 실행·진단 스크립트

| 파일 | 역할 |
|---|---|
| `scripts/plan_from_live_vision.py` | 파일 기반 단독 진단용으로 최신 YOLO scene을 읽어 T자 후진주차 경로를 한 번 생성하고 검증 결과를 저장합니다. 통합 실행은 이 파일의 planner 함수를 메모리에서 재사용합니다. |
| `scripts/read_live_vision_scene.py` | 현재 planner에 전달될 차량 pose와 주차면 goal을 출력합니다. |
| `scripts/test_live_vision_scene.py` | ego 선택, 수동 heading, 주차면 점유 및 stale scene 차단을 회귀 검사합니다. |
| `scripts/test_t_parking.py` | staging과 마지막 후진을 포함한 T자 주차 경로를 회귀 검사합니다. |
| `scripts/test_plan_from_live_vision.py` | vision scene부터 Hybrid A* 저장까지 통합 흐름을 회귀 검사합니다. |
| `scripts/test_trajectory_profile.py` | 곡률 기반 속도와 기어 전환 정지점 계산을 검사합니다. |
| `scripts/test_trajectory_validator.py` | 정상·비정상 trajectory의 안전 차단 조건을 검사합니다. |
| `scripts/test_path_publisher.py` | 경로·차량 pose ROS message 변환과 stale 차단을 검사합니다. |
| `scripts/test_fixed_mission_routes.py` | 출발지·주차면·충전면·도착지 사이의 고정 단방향 mission route를 생성하고 Camera BEV에 표시합니다. |
| `scripts/test_fixed_route_selector.py` | 검출된 현재 차량 pose에서 호환되는 고정 경로를 선택하고 가장 가까운 지점부터 이어지는지 검사합니다. |

## 고정 mission route

`config/fixed_mission_routes.yaml`은 다음 이동 관계를 허용합니다.

- `START` → `P10`~`P6` 또는 `C1`~`C2`
- `P10`~`P6` → `C1`~`C2`
- `C1`~`C2` → `P1`~`P5`
- `P1`~`P5` → `EXIT`

슬롯에서 출차할 때는 검증된 주차 maneuver를 반대로 따라 전진하고, 목표
슬롯에서는 도로 중앙선 staging에서 단일 후진 maneuver로 진입합니다.

```bash
python3 scripts/test_fixed_mission_routes.py --source START --target C2
python3 scripts/test_fixed_mission_routes.py --source P6 --target C1
python3 scripts/test_fixed_mission_routes.py --check-all
python3 scripts/test_fixed_mission_routes.py --generate-all
```

기본 출력은 제어 입력용 CSV이며, 진단 이미지가 필요할 때만
`--save-image`를 추가합니다.

실시간 route selector는 localization의 rear-axle pose로 현재 위치가
`START` 또는 어느 주차면인지 판별한 뒤, 해당 source에서 목표로 가는 전체
고정 경로를 선택합니다. 차량이 도로 이동 중인 `TRANSIT` 상태에서는 새로운
경로 생성을 시작하지 않습니다.

YOLO vehicle center가 주차면 안에 있으면 그 slot의 고정 goal yaw를 사용하고,
그 외 초기 위치에서는 START yaw를 사용합니다. 별도의 ego-front 클릭 없이
선택된 경로는 `/pinkk/planned_path`와 `/pinkk/planned_trajectory`로 바로
발행되며 trajectory 행렬 필드는 `x_m, y_m, yaw_rad, direction` 네 개입니다.

## 통합 실행과 진단 출력

| 항목 | 역할 |
|---|---|
| 기본 통합 실행 | Camera localization 창에서 heading 지정 후 Hybrid A*를 시작하고, 검증 trajectory를 `/pinkk/planned_path`, `/pinkk/planned_trajectory`로 직접 발행합니다. 별도 trajectory 파일은 저장하지 않습니다. |
| `/pinkk/vehicle_pose` | 같은 프로세스가 최신 rear-axle pose를 직접 발행합니다. |
| `output/live_*.csv/json/png` | 파일 기반 진단 스크립트 또는 `write_runtime_files: true`일 때만 생성하는 선택적 결과입니다. 제어기 연동에는 사용하지 않습니다. |

## 문서

| 파일 | 역할 |
|---|---|
| `DEVELOPMENT_MEMO.md` | 구현 변경, 판단 근거, 회귀 결과와 다음 작업을 기록합니다. |
| `THIRD_PARTY_NOTICES.md` | Reeds–Shepp 구현 등 외부 코드의 출처와 라이선스를 기록합니다. |
| `README.md` | 이 폴더의 파일 역할을 설명합니다. |
