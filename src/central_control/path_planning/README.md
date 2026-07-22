# Path Planning

Python 기반 자율주행 주차 경로계획 모듈입니다. LiDAR occupancy grid의 2D A* 비교 기능과 함께, 차량 yaw·조향·전후진·회전 직사각형 footprint 충돌을 고려하는 **Hybrid A***를 구현합니다.

## 구성

- `config/map_config.yaml`: 지도, BEV, LiDAR 및 정합 파일 설정
- `config/vehicle_config.yaml`: 차량 제원과 조향 제약
- `config/planner_config.yaml`: A*, Hybrid A* 및 analytic expansion 파라미터
- `src/occupancy_grid.py`: ROS map_server 형식 PGM/YAML 로더
- `src/coordinate_transform.py`: BEV pixel, world cm, grid 좌표 변환
- `src/astar_planner.py`: 4/8방향 2D A* 탐색
- `src/hybrid_astar_planner.py`: 차량 yaw·조향·전후진을 고려한 Hybrid A*
- `src/reeds_shepp.py`: Hybrid A* 목표 연결에 사용할 Reeds–Shepp 경로 생성기
- `src/path_smoothing.py`: 기어 전환을 보존하는 cubic spline 경로 smoothing
- `src/trajectory_profile.py`: Hybrid pose의 곡률·목표속도·각속도·정지점 생성
- `src/trajectory_validator.py`: 제어기 전달 전 trajectory fail-closed 안전 검사
- `src/vision_scene_input.py`: 최신 카메라 scene의 유효성·시간·지도 범위를 검사하는 입력 gate
- `src/visualization.py`: 지도와 경로의 OpenCV 시각화
- `scripts/`: 지도 로딩 및 A* 실행 스크립트
- `output/`: 실행 결과 이미지

좌표는 A*와 occupancy grid에서 `(x, y) = (column, row)`를 사용하며 실제 배열 접근은 `grid[y, x]`입니다. BEV 원점은 좌상단이고 y가 아래로 증가하지만, world cm 원점은 좌하단이고 y가 위로 증가합니다.

`OccupancyGridMap`은 기본적으로 `block_outside_area=True`를 사용합니다. 외벽의 작은 끊김을 morphology close로 막은 임시 mask에서 테두리와 연결된 자유 공간을 찾고, 그 외부 영역을 최종 grid의 장애물로 처리합니다. morphology로 두께워진 임시 장애물은 최종 grid에 복사하지 않습니다.

## Obstacle inflation

2D A*는 차량을 크기가 없는 하나의 점으로 보므로, 원본 occupancy grid만 사용하면 벽과 주차선 바로 옆으로 경로가 생성될 수 있습니다. 이를 방지하기 위해 차량 반폭과 안전 마진을 고려한 obstacle inflation을 적용합니다.

현재 2D A* 테스트의 기본 inflation radius는 **7 cm**입니다. 원본 grid는 유지하고, 원형 타원 kernel로 장애물을 팽창한 별도 grid에서 A*를 수행합니다. Hybrid A*는 차량 크기를 inflation으로 대신하지 않고 실제 회전된 rectangle footprint를 사용합니다. 좁은 실차 주차면 시험에서는 별도 팽창을 더하지 않도록 `footprint_safety_margin_cm: 0.0`이며, 차량 외곽이 원본 장애물에 닿는 pose는 그대로 충돌 처리합니다.

## 요구 환경

- Python 3.10 이상
- `numpy`
- `opencv-python`
- `pyyaml`

필요한 경우 다음과 같이 설치합니다.

```bash
python3 -m pip install numpy opencv-python pyyaml
```

기본 지도 파일은 `../camera_tools/first_map/my_test_map0710.pgm` 및 `my_test_map0710.yaml`입니다. PGM이 없고 YAML의 `image` 항목이 실제 PNG 파일을 가리키면 테스트 스크립트는 그 이미지를 대신 사용합니다. 이미지 밝기가 100 미만인 셀은 장애물(100), 나머지는 자유 공간(0)으로 변환합니다.

## 실행 방법

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/test_map_load.py
python3 scripts/test_astar.py
python3 scripts/test_astar_overlay.py
python3 scripts/test_astar_on_camera_bev.py
python3 scripts/click_astar_on_camera_bev.py
python3 scripts/test_hybrid_astar.py
python3 scripts/test_hybrid_astar_analytic.py
python3 scripts/test_reeds_shepp.py
python3 scripts/test_reeds_shepp_collision.py
python3 scripts/test_trajectory_profile.py
python3 scripts/test_trajectory_validator.py
python3 scripts/test_live_vision_scene.py
python3 scripts/read_live_vision_scene.py
python3 scripts/test_plan_from_live_vision.py
python3 scripts/plan_from_live_vision.py
python3 scripts/click_hybrid_astar_on_camera_bev.py
```

생성되는 파일은 다음과 같습니다.

- `output/debug_occupancy_grid.png`: 흰색 자유 공간과 검은색 장애물로 표현한 occupancy grid
- `output/debug_inflated_grid.png`: 7 cm 안전 마진으로 팽창한 occupancy grid
- `output/astar_result.png`: inflated grid 위의 경로(빨강), 시작점(초록), 목표점(파랑)
- `output/astar_inflation_comparison.png`: 원본 장애물(검정), inflation 영역(회색), A* 경로를 함께 표시
- `output/astar_on_overlay.png`: camera/LiDAR rigid overlay 위에 표시한 A* 경로
- `output/astar_on_camera_bev.png`: 역 affine 변환으로 Camera BEV 위에 표시한 A* 경로
- `output/click_astar_on_camera_bev.png`: Camera BEV에서 클릭한 시작점·목표점으로 생성한 A* 결과
- `output/path_lidar_grid.csv`: A*가 생성한 원본 LiDAR grid 경로
- `output/path_camera_bev.csv`: 역 affine 변환된 Camera BEV pixel 경로
- `output/path_world_cm.csv`: 제어 테스트에 사용할 기본 world cm 경로
- `output/path_world_cm.json`: world cm 경로와 frame·resolution·planner metadata
- `output/path_world_cm_raw.csv`: `path_world_cm.csv`와 동일한 A* raw path 백업
- `output/path_world_cm_simplified.csv`: RDP로 단순화한 제어팀 전달용 1차 추천 경로
- `output/path_world_cm_simplified.json`: RDP 설정 metadata와 단순화 경로
- `output/click_hybrid_astar_on_camera_bev.png`: 클릭한 start/goal pose로 생성한 Hybrid A* 결과
- `output/hybrid_path_world_cm.csv`: Hybrid A*의 pose·곡률·목표속도·각속도·정지 플래그를 포함한 제어용 trajectory
- `output/hybrid_path_camera_bev.csv`: Hybrid pose를 Camera BEV pixel로 변환한 경로
- `output/hybrid_path_world_cm.json`: Hybrid pose 경로와 frame·planner metadata
- `output/hybrid_smoothing_stats.json`: raw·최종 경로의 smoothing 비교 통계와 fallback 사유

`test_astar.py`는 기본 시작점 `(20, 20)`과 목표점 `(200, 180)`을 사용합니다. 점이 장애물이면 반경 30셀 안에서 가장 가까운 자유 셀을 찾고, 목표가 지도 밖이면 지도 크기에 맞춰 안쪽으로 보정합니다. 연결 가능한 경로가 없으면 이미지 대신 명확한 실패 메시지를 출력합니다.

`test_astar_overlay.py`는 inflated grid에서 생성한 grid 경로를 world cm로 변환한 다음, 좌하단 world 원점과 `8 px/cm` 비율을 사용해 BEV pixel로 변환합니다. 정합 이미지가 기준 크기 `1600×800`과 다르면 경고하며, 이미지 범위 밖의 경로점은 그리지 않고 개수를 출력합니다. 현재 `camera_lidar_rigid_overlay.png`처럼 정합 결과가 LiDAR grid 크기로 저장된 경우에는 grid 셀이 이미 오버레이 pixel과 일치하므로 직접 좌표 정렬을 사용합니다.

`test_astar_on_camera_bev.py`는 `camera_to_lidar_rigid_registration.npz`의 `affine_matrix`(방향: Camera BEV pixel → LiDAR pixel)를 `cv2.invertAffineTransform`으로 뒤집어 LiDAR grid의 A* 경로를 Camera BEV pixel로 투영합니다. `bev_result.png`, `latest_bev.png`, `camera_bev.png`, `live_bev.png`, `undistorted_bev.png` 순서로 이미지를 찾으며, 모두 없으면 경고 후 흰색 `1600×800` 배경을 사용합니다.

`click_astar_on_camera_bev.py`는 `camera_bev.png`의 첫 번째 클릭을 시작점, 두 번째 클릭을 목표점으로 사용합니다. Camera pixel을 정방향 affine로 LiDAR grid에 변환해 inflated grid에서 A*를 수행하고, 역 affine로 경로를 Camera BEV에 다시 표시합니다. `r`은 초기화, `s`는 결과 이미지와 좌표 파일 저장, `q` 또는 `ESC`는 종료입니다.

## 제어 경로 파일

Camera BEV에서 start 클릭, goal 클릭, 경로 확인 후 `s`를 누르면 이미지와 좌표 파일을 함께 저장합니다. `path_world_cm.csv`는 A* raw path이고 `path_world_cm_raw.csv`는 그 내용의 백업입니다. **`output/path_world_cm_simplified.csv`를 제어팀 전달용 1차 추천 경로로 사용합니다.**

단순화는 Ramer-Douglas-Peucker(RDP) 알고리즘을 사용하며 기본 epsilon은 **3 cm**입니다. simplified path의 yaw는 남은 각 경로점에서 다음 점을 향하도록 다시 계산합니다. 아직 Hybrid A*가 아니므로 차량 회전반경과 후진은 반영되지 않습니다.

- 좌표 단위: cm
- `yaw_rad`, `yaw_deg`: 각 경로점에서 다음 경로점을 향하는 진행 방향
- `direction`: 현재 2D A*에서는 모두 전진 `1`
- Hybrid A* 적용 후 `yaw`와 `direction`은 차량 기구학을 반영한 값으로 변경 예정

## Hybrid A* 및 헤딩 지정

Hybrid A*는 탐색 상태를 연속 `(x_cm, y_cm, yaw, direction)` pose로 확장합니다. wheelbase와 steering angle을 사용한 kinematic bicycle model로 3 cm motion primitive를 생성하므로, 2D A*처럼 움직임 방향이 즉시 45도씩 변하지 않습니다. 조향각 후보는 기본 `-30°`부터 `30°`까지 **5° 간격**이고, 전진과 후진을 모두 탐색합니다. 인접 primitive 사이의 조향 변화는 최대 10°로 제한하여 `-30°`에서 `+30°`로 즉시 바뀌는 경로를 생성하지 않습니다.

탐색 성능을 위한 3 cm motion primitive와 제어기에 전달하는 경로 간격을 분리합니다. 경로를 찾은 후에 각 primitive의 조향각과 `direction`을 유지한 채 bicycle model로 다시 적분하여 **0.5 cm 간격**의 고밀도 pose를 생성합니다. 따라서 단순 직선 보간과 달리 곡선을 따라 `x`, `y`, `yaw`가 점진적으로 변합니다. 간격은 `config/planner_config.yaml`의 `path_output_step_cm`으로 조정합니다.

## 제어용 trajectory 속도 프로파일

Hybrid A*가 생성한 고밀도 pose의 가상 조향각을 다음 식으로 곡률로 변환합니다.

```text
curvature_1pm = tan(steer_rad) / wheelbase_m
  target_angular_z_radps = target_speed_mps * curvature_1pm
```

`target_speed_mps`는 signed 속도이므로 전진은 양수, 후진은 음수입니다. 직선에서는 빠르고 차량의 최대 가상 조향각 `30°`를 기준으로 곡률이 커질수록 감속합니다. 0.5 cm 경로 간격을 사용한 전방·후방 패스로 가속도와 감속도를 제한합니다. 시작, 도착, 전진↔후진 전환 직전 pose는 `stop_required=1`, `target_speed_mps=0` 조건을 가집니다.

초기 실차 시험은 다음 보수적 제한을 사용합니다.

```yaml
trajectory_profile:
  max_forward_speed_mps: 0.05
  max_reverse_speed_mps: 0.03
  min_curve_forward_speed_mps: 0.02
  min_curve_reverse_speed_mps: 0.015
  max_angular_speed_radps: 0.5
  max_acceleration_mps2: 0.05
  max_deceleration_mps2: 0.05
```

`hybrid_path_world_cm.csv` 및 JSON 경로 항목에는 다음 컬럼이 추가됩니다.

```text
curvature_1pm
target_speed_mps
target_angular_z_radps
stop_required
```

## 제어기 전달 전 trajectory 안전 검사

속도 프로파일 생성이 끝나면 `validate_trajectory()`를 통과한 경로만 테스트 결과로 등록합니다. 검사 실패 시 경로 목록을 비워 `s` 저장을 차단하며, 향후 ROS2 publisher도 동일한 `valid` 결과를 전송 조건으로 사용합니다. Camera BEV 클릭은 현재 start/goal 입력을 위한 테스트 수단일 뿐이며 validator는 입력 방식과 독립적입니다.

다음 조건을 검사합니다.

- 빈 경로 및 `NaN`·무한대 값
- direction과 signed 속도의 일치 여부
- 전진 `0.05 m/s`, 후진 `0.03 m/s`, 각속도 `0.5 rad/s` 제한
- 조향각 `±30°`와 거리 기준 조향 변화율
- 곡률과 `tan(steer) / wheelbase`의 일치 여부
- 각속도와 `speed × curvature`의 일치 여부
- 경로점 간격 `0.5 cm`, yaw 변화 및 실제 이동방향 연속성
- 가속도·감속도 `0.05 m/s²` 제한
- 시작·도착·전후진 전환 직전의 필수 정지
- 모든 pose의 차량 rectangle footprint 충돌

실패 시 오류 코드와 pose index를 출력하고 경로 전달을 차단합니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/test_trajectory_validator.py
```

차량 pose의 기준점은 **뒷바퀴 축 중심**입니다. 실차 측정값을 반영한 차량 `12×10 cm`, wheelbase `8 cm`, rear overhang `2 cm`로 회전된 직사각형이 덮는 모든 occupancy cell을 검사합니다. 현재 추가 안전마진은 0cm이므로 직선 벽에서 경로 중심선까지 필요한 최소 거리는 차량 반폭인 5cm입니다. motion primitive 중간도 0.5 cell 이하 간격으로 검사하므로, 회전 중 차체 모서리가 벽을 건너뛰는 경로를 차단합니다. 클릭한 start/goal이 기준점으로는 free여도 차체가 장애물을 침범하면 30 cm 반경 내의 가장 가까운 footprint-valid pose로 보정합니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/click_hybrid_astar_on_camera_bev.py
```

마우스 클릭 순서는 다음과 같습니다.

1. 시작 위치
2. 시작 차량이 바라볼 방향의 점
3. 목표 위치
4. 목표 차량이 바라볼 방향의 점

`r`로 클릭과 경로를 초기화하고, `s`로 Hybrid 결과 이미지와 CSV/JSON을 저장하며, `q` 또는 `ESC`로 종료합니다. `hybrid_path_world_cm.csv`의 yaw는 단순화 후 재계산한 값이 아니라 planner가 자동차 모델로 전개한 각 pose의 헤딩입니다. `direction` 값은 전진 `1`, 후진 `-1`입니다.

## 상단 카메라 실시간 start/goal 입력

상단 카메라 localization 프로세스는 YOLO 차량 segmentation 결과와 고정 주차면을 정합해 다음 정보를 `output/live_vision_scene.json`에 저장합니다.

- ego 차량의 Camera BEV 중심과 LiDAR 중심
- 차량 중심에서 4 cm 뒤로 보정한 rear-axle `(x_cm, y_cm, yaw_rad)` pose
- 모든 주차면의 Camera BEV·LiDAR 중심과 차량 mask 점유율
- 설정된 목표 주차면과 서로 180° 다른 두 rear-axle goal pose 후보

현재 통합 테스트 설정은 `../config/yolo/realtime_localization.yaml`의 `parking_assignment.active_phase: entry_to_parking`을 사용합니다. `config/map/parking_points.json`의 입구 BEV 좌표를 기준으로 **P6~P10 중 비어 있는 칸을 먼 순서**로 정렬하고, 첫 후보만 planner에 전달합니다. 따라서 가장 먼 칸이 점유되면 다음 후보로 즉시 넘어갑니다. 현재 지도에서는 대체로 `P6 → P7 → P8 → P9 → P10` 순서이며, 실제 순서는 각 칸 중심과 입구 사이의 BEV 직선거리로 결정됩니다. 후보 순서는 `output/live_vision_scene.json`의 `candidate_slot_names`에 저장됩니다.

`charge_to_exit` 단계의 C1/C2 충전 후 P1~P5 배정 규칙은 설정에 미리 등록해 두었지만, 에피소드 상태 전환은 아직 구현하지 않았습니다. 현 단계에서는 입구 차량의 첫 주차 배정만 동작합니다.

### P6 후진주차 구현 메모

2026-07-21 점검 결과, 기존 P6 경로 245점 중 242점이 전진이고 마지막 20점도 모두 `direction=1`이었습니다. Hybrid A*의 goal 판정이 `x·y·yaw`만 검사하고 후진 비용을 1.5배로 두기 때문에, 같은 자세에 전진으로 도착할 수 있으면 전진주차를 선택한 것이 원인이었습니다.

자동 카메라 경로에는 `config/planner_config.yaml`의 `automatic_parking` 조건을 적용합니다.

```yaml
automatic_parking:
  required_final_direction: -1
```

기존에는 목표 12cm 앞에 고정 approach pose를 만들고 마지막 구간을 반드시 직선 후진하도록 강제했습니다. 이 조건은 실제 차체가 통과할 수 있는 주차면도 과도하게 차단할 수 있어 설정과 실행 코드에서 모두 제거했습니다. 현재 실시간 자동 계획은 주차칸 polygon의 **짧은 두 변**을 입구 후보로 보고, map에서 통로 여유가 더 큰 쪽을 입구로 자동 선택합니다. 차량 앞은 선택된 입구를 향하고, rear axle은 주차칸 안쪽에 놓이며 마지막 이동 방향은 `-1` 후진으로 제한합니다.

자동 경로 생성기는 주차칸 이름으로 분기하지 않습니다. 모든 주차면에 차량 제원, 최소 회전반경, 조향 한계, occupancy map, 목표 pose의 동일한 조건을 적용합니다. 최종 trajectory validator는 `FINAL_DIRECTION_MISMATCH`를 검사합니다.

### P5 진입 차단 진단 메모

2026-07-21 초기 진단에서는 2cm 추가 inflation과 12cm 고정 직선 후진 corridor가 결합되어 좁은 주차면을 과도하게 차단했습니다. 현재는 추가 margin을 0cm로 낮추고 고정 corridor 코드를 제거했습니다. 변경 후 동일 회귀에서 P10은 약 **0.21초**, P6은 약 **0.16초**, C1은 약 **0.56초**에 마지막 후진 방향을 만족하는 경로를 생성했습니다. P8/P9는 footprint 충돌 즉시 차단 상태에서는 벗어났지만 짧은 후보 제한 안에 조향 스무딩을 통과하지 못해 탐색·스무딩 개선 대상으로 남아 있습니다.

경로계획 쪽의 `load_vision_planning_request()`는 `planning_ready=true`인 최신 scene만 읽습니다. 기본 허용 나이는 **0.5초**이며 파일이 오래됐거나, ego 선택·차량 앞뒤가 모호하거나, start/goal이 지도 밖이면 `VisionSceneUnavailable`로 입력을 거부합니다. 카메라 프로세스와 별도로 아래 명령을 실행하면 실제로 전달될 start/goal을 확인할 수 있습니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/read_live_vision_scene.py
```

mock 회귀 테스트는 다중 차량 중 초기 ego 선택, 점유/빈 주차면 판정, 두 goal heading, stale 입력 차단 및 모호한 scene 차단을 함께 검사합니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/test_live_vision_scene.py
```

차량 heading은 기본적으로 수동 지정합니다. localization 창에서 `h`를 누르고 검출 차량의 앞쪽을 클릭합니다. `x`는 수동 heading 초기화입니다. 클릭 전에는 `planning_ready=false`이며 클릭한 방향은 다시 지정할 때까지 고정됩니다. 기존 `--initial-yaw-deg` 숫자 방식은 화면이 없는 테스트를 위해 유지합니다. 아래 one-shot 자동 계획까지 연결했지만 ROS 2 토픽 및 `/home/junguk/pinky_ctrl` 제어기 전송은 아직 수행하지 않습니다.

### 차량 검출 후 자동 Hybrid A* 생성

localization 프로세스가 실행 중일 때 아래 스크립트는 최대 10초 동안 fresh `planning_ready` scene을 기다립니다. 차량이 발견되면 현재 rear-axle pose를 start로 사용하고, 선택된 빈 주차칸의 자동 입구 방향을 향한 하나의 goal pose에 대해 후진주차 경로를 한 번 계획하고 종료합니다. 현재 전체 계획과 각 후보의 기본 시간 제한은 `0`(무제한)입니다. 탐색 중에는 1,000 node마다 진행 상황을 출력하며, 비정상 상태공간 폭증을 막기 위한 후보당 확장 노드 상한은 유지합니다.

필요할 때만 `--planning-timeout-sec`와 `--candidate-timeout-sec`에 양수를 지정해 시간 제한을 다시 적용할 수 있습니다.

클릭 기반 Hybrid A*는 목표 heading을 포함한 3cm 주변 pose 탐색을 유지합니다. 반면 실시간 자동 계획은 polygon과 map 여유로 계산한 하나의 주차칸 입구 heading만 사용하며 주변 pose 반복 탐색을 하지 않습니다. 일반 Hybrid A* goal 도달 경로도 Reeds–Shepp 도달 경로와 동일하게 곡률 smoothing을 적용하므로 3cm primitive의 조향 변화가 0.5cm 출력에서 순간 변화로 남지 않게 검사합니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/plan_from_live_vision.py
```

### T자 후면주차 방식

P6~P10 같은 통로 직각 주차면은 지도 전체에서 한 번에 주차 pose를 찾지 않는다. 먼저 주차칸 입구 앞 통로의 **staging pose**까지 Hybrid A*로 접근하고, 이 지점에서 정지·조향을 재설정한 뒤 두 번째 짧은 Hybrid A*로 전진·후진 T자 maneuver를 생성한다. final pose까지의 마지막 후진 거리는 최소 **10cm**로 제한한다.

- staging 앞 경로와 주차 maneuver는 각각 차량 footprint, yaw, steering, 전후진을 검사한다.
- staging에는 연속된 두 stop point가 저장된다. 차량은 이 위치에서 정지한 뒤 조향을 바꾸고 maneuver를 시작한다.
- staging 연결은 위치 1cm, yaw 5° 이내로 엄격하게 맞춘다. 통로 접근 탐색은 후보당 3초, 최대 4개의 staging 후보로 제한한다. 모두 실패하면 이전 경로를 저장하지 않고 실패를 출력한다.

회귀 테스트:

```bash
python3 scripts/test_t_parking.py
```

### ROS 2 경로 토픽 발행

검증을 통과한 `output/live_hybrid_path_world_cm.json`만 ROS 2로 발행한다. publisher는 파일 변경을 감시하며 새 경로가 저장될 때마다 마지막 경로를 다시 발행한다. QoS는 `TRANSIENT_LOCAL`이므로 제어기가 나중에 시작해도 마지막 경로를 받을 수 있다.

- `/pinkk/planned_path` (`nav_msgs/Path`): Pure Pursuit용 pose 경로. 좌표 단위는 **m**, frame은 `lidar_map`이다.
- `/pinkk/planned_trajectory` (`std_msgs/Float64MultiArray`): 각 점의 `[x_m, y_m, yaw_rad, direction, target_speed_mps, steer_rad, stop_required]` 순서 제어 정보다. `direction=1`은 전진, `-1`은 후진이다.
- `/pinkk/vehicle_pose` (`geometry_msgs/PoseStamped`): 상단 카메라가 검출한 현재 rear-axle pose다. 단위는 **m**, frame은 `lidar_map`이다. 차량 검출·heading이 모호하거나 scene이 0.5초 이상 오래되면 발행하지 않는다.

```bash
cd ~/PINKK
source /opt/ros/<ros_distro>/setup.bash
python3 -m src.central_control.overhead_vision.path_planning.path_publisher
```

토픽 확인:

```bash
ros2 topic echo --once /pinkk/planned_path
ros2 topic echo --once /pinkk/planned_trajectory
```

현재 차량 위치 publisher는 localization 실행 중 별도 터미널에서 시작한다.

```bash
cd ~/PINKK
source /opt/ros/jazzy/setup.bash
python3 -m src.central_control.overhead_vision.path_planning.vehicle_pose_publisher
```

```bash
ros2 topic echo /pinkk/vehicle_pose
```

경로 publisher는 제어 명령을 내리지 않는다. 실제 PP/PI 제어기는 토픽을 구독해 별도로 구동해야 한다.

처리 순서는 다음과 같습니다.

1. 0.5초 이내의 vision scene과 지도 범위를 검사
2. start/goal의 회전 차량 footprint가 충돌하면 30 cm 안에서 가까운 유효 pose로 보정
3. polygon의 짧은 두 변 중 map 통로 여유가 큰 변을 주차칸 입구로 선택
4. 차량 앞이 입구를 향하는 rear-axle goal을 만들고 Hybrid A*로 전진·후진 조합 탐색
5. 최종 이동 방향이 후진인지 확인
5. 0.5 cm 이하 간격의 곡률 smoothing과 속도 프로파일 생성
6. footprint 충돌과 trajectory validator 통과 시에만 파일 저장

출력 파일:

- `output/live_hybrid_path_world_cm.csv`: 추후 제어기 연결에 사용할 검증된 trajectory
- `output/live_hybrid_path_camera_bev.csv`: 같은 경로의 Camera BEV 좌표
- `output/live_hybrid_path_world_cm.json`: 검출 frame, 주차면, start/goal, planner·검증 및 `parking_maneuver` metadata
- `output/live_camera_bev.png`: localization이 원자적으로 갱신하는 최신 원본 Camera BEV
- `output/live_hybrid_path_on_camera_bev.png`: 빨간 경로·초록 start·파란 goal heading overlay
- `output/live_hybrid_planning_status.json`: 최신 성공 여부 또는 차단 이유

localization은 scene JSON을 공개하기 직전에 동일 frame의 Camera BEV를 임시 PNG에서 `live_camera_bev.png`로 원자 교체합니다. 자동 planner는 이 이미지를 우선 사용하며, 파일이 없을 때만 기존 `camera_bev.png`를 사용하고 경고합니다. 이미지 범위 밖 경로점은 선을 갑자기 잇지 않고 건너뛰며 개수를 JSON의 `visualization.out_of_bounds_path_points`에 기록합니다.

검출 누락, stale 좌표, 모호한 헤딩, 경로 탐색 실패, smoothing 실패 또는 validator 실패 시 이전 자동 경로 파일과 overlay를 삭제합니다. 따라서 단순히 Hybrid A*가 경로를 찾았다는 이유만으로 제어용 파일이 남지 않습니다. 이번 구현은 **한 scene에서 한 번 계획하는 기본 파이프라인**이며, 지속 재계획과 제어기 전송은 아직 하지 않습니다.

검증된 현재 자동 배정 목표 경로가 생성되면 localization 프로세스가 `live_hybrid_path_world_cm.json` 변경을 감지해 현재 창에 굵은 빨간 polyline으로 표시합니다. 차량이 저장 경로의 start에서 8cm 이상 이동하면 오래된 경로를 숨깁니다. 화면에 선이 없으면 아직 planner를 실행하지 않았거나, 경로가 validator에서 차단됐거나, 실행 중인 localization이 변경 전 설정을 읽은 상태인지 확인합니다.

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/test_plan_from_live_vision.py
```

회귀 테스트는 일반 경로의 smoothing·validator·CSV/JSON 저장과 함께 C1/P6의 전진 approach→정지→12cm 후진 진입을 검사합니다. P6에서는 충돌하는 primary approach를 거부하고 alternative heading을 선택하는지도 확인합니다.

## Reeds–Shepp 목표 연결기

기존 Hybrid A*를 교체하지 않고 목표 pose 부근의 탐색을 보조하기 위한 독립 Reeds–Shepp 생성기를 추가했습니다. 차량 wheelbase `8 cm`와 최대 가상 조향각 `30°`를 기준으로 최소 회전반경은 약 **13.86 cm**이며, 전진·후진을 포함한 `L`(좌회전), `R`(우회전), `S`(직진) 후보 중 가장 짧은 경로를 **0.5 cm 간격**으로 출력합니다.

수학적인 경로 생성과 끝 pose·샘플 간격·전후진 전환을 독립 테스트합니다. 생성된 모든 0.5 cm pose는 기존 `HybridAStarPlanner`의 회전 직사각형 footprint 판정을 그대로 사용해 검사합니다. `first_path_collision_index()`는 최초 충돌 pose를 반환하고 `is_path_collision_free()`는 전체 경로 통과 여부를 반환하므로, 차량 일부가 지도 밖으로 나가거나 경로 중간에서 장애물을 침범하는 경우도 검출합니다.

Hybrid A*는 목표와의 직선거리가 기본 **30 cm** 이내가 되면 Reeds–Shepp analytic expansion을 시도합니다. 후보를 길이순으로 검사하여 footprint 충돌이 없는 첫 경로만 기존 탐색 경로 뒤에 붙입니다. 모든 후보가 막히면 경로계획을 실패시키지 않고 기존 motion primitive 탐색을 계속합니다. 연결 성공 시 허용 오차 부근에서 끝내지 않고 요청한 goal의 `x`, `y`, `yaw`에 정확히 도달합니다.

analytic 경로의 물리적 최소 회전반경은 wheelbase와 최대 조향각으로 계산한 값보다 작아지지 않으며, 실차 설정은 `min_turning_radius_cm: 14.0`입니다. 실제 analytic 후보는 smoothing에 필요한 곡률 여유 `4 cm`를 더한 **18 cm** 회전반경을 사용합니다. 활성화 여부와 시도 거리, 곡률 여유는 `config/planner_config.yaml`의 `analytic_expansion_enabled`, `analytic_expansion_distance_cm`, `analytic_turning_radius_margin_cm`으로 조정합니다. 기반 수식 구현의 출처와 MIT 라이선스는 `THIRD_PARTY_NOTICES.md`에 기록했습니다.

## 곡률 및 조향 변화 smoothing

충돌 안전 analytic 경로를 기어가 같은 구간별로 나눈 뒤 clamped cubic spline으로 다시 생성합니다. spline은 위치뿐 아니라 시작·목표 접선도 고정하므로 요청한 start/goal의 `x`, `y`, `yaw`를 유지합니다. 전진↔후진 cusp는 두 spline 구간이 공유하는 고정점으로 남기며, 해당 위치에서는 trajectory profile이 정지를 요구하므로 정지 후 조향 방향을 바꿀 수 있습니다.

기본 control knot 간격은 **6 cm**이고 최종 경로는 **0.5 cm 이하** 간격으로 다시 샘플링합니다. 각 spline 미분값으로 yaw, 곡률 및 가상 조향각을 재계산한 후 다음 조건을 모두 검사합니다.

- 최대 가상 조향각 `30°` 이하
- 조향 변화율 `10° / 3 cm` 이하
- 시작·목표 위치와 yaw 유지
- 모든 pose의 회전 직사각형 footprint 충돌 없음
- 출력 pose 간격 `0.5 cm` 이하

검사에 실패하면 spline 결과를 폐기하고 이미 충돌 검사를 통과한 raw Hybrid A* + Reeds–Shepp 경로를 그대로 사용합니다. `click_hybrid_astar_on_camera_bev.py` 실행 결과의 `Goal connection` 항목에서 `curvature-smoothed path accepted` 또는 raw fallback 사유를 확인할 수 있습니다.

현재 Camera BEV 클릭은 경로계획 기능을 확인하기 위한 테스트 입력으로만 유지합니다. 테스트 경로 확인 후 `s`를 누르면 기존 이미지·CSV·JSON과 함께 `output/hybrid_smoothing_stats.json`을 저장합니다. 통계 저장 함수는 클릭 스크립트와 분리되어 있어 추후 차량 위치 인식 기반 자동 경로 생성에서도 그대로 사용할 수 있습니다.

통계 JSON은 다음 항목을 포함합니다.

- smoothing 시도 및 채택 여부, raw fallback 사유
- raw/candidate/final pose 개수와 경로 길이
- 최대 경로점 간격과 최대 절대 조향각
- 같은 기어 구간의 최대 조향 변화량
- 전진↔후진 기어 전환 횟수
- spline knot 간격과 analytic 회전반경

```bash
cd ~/PINKK/src/central_control/path_planning
python3 scripts/test_reeds_shepp.py
python3 scripts/test_reeds_shepp_collision.py
python3 scripts/test_hybrid_astar_analytic.py
```

## 다음 단계

- 실제 설치 위치에서 ego의 초기 BEV 중심과 LiDAR 기준 앞 방향 측정
- 차량 앞/뒤를 구분할 marker 또는 front class를 추가해 180° heading 모호성 제거
- 여러 빈 주차면을 경로 가능성까지 비교해 최종 goal을 선택
- 정지 상태로 안정된 여러 프레임에서만 one-shot planner를 호출하는 coordinator 추가
- 이후 `/home/junguk/pinky_ctrl` 입력 형식에 맞춰 validator 통과 trajectory만 ROS 2로 전달
