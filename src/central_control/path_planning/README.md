# Path Planning

Python 기반 자율주행 주차 경로계획 모듈입니다. LiDAR occupancy grid의 2D A* 비교 기능과 함께, 차량 yaw·조향·전후진·회전 직사각형 footprint 충돌을 고려하는 **Hybrid A***를 구현합니다.

## 구성

- `config/map_config.yaml`: 지도, BEV, LiDAR 및 정합 파일 설정
- `config/vehicle_config.yaml`: 차량 제원과 조향 제약
- `config/planner_config.yaml`: A* 및 향후 Hybrid A* 파라미터
- `src/occupancy_grid.py`: ROS map_server 형식 PGM/YAML 로더
- `src/coordinate_transform.py`: BEV pixel, world cm, grid 좌표 변환
- `src/astar_planner.py`: 4/8방향 2D A* 탐색
- `src/hybrid_astar_planner.py`: 차량 yaw·조향·전후진을 고려한 Hybrid A*
- `src/visualization.py`: 지도와 경로의 OpenCV 시각화
- `scripts/`: 지도 로딩 및 A* 실행 스크립트
- `output/`: 실행 결과 이미지

좌표는 A*와 occupancy grid에서 `(x, y) = (column, row)`를 사용하며 실제 배열 접근은 `grid[y, x]`입니다. BEV 원점은 좌상단이고 y가 아래로 증가하지만, world cm 원점은 좌하단이고 y가 위로 증가합니다.

`OccupancyGridMap`은 기본적으로 `block_outside_area=True`를 사용합니다. 외벽의 작은 끊김을 morphology close로 막은 임시 mask에서 테두리와 연결된 자유 공간을 찾고, 그 외부 영역을 최종 grid의 장애물로 처리합니다. morphology로 두께워진 임시 장애물은 최종 grid에 복사하지 않습니다.

## Obstacle inflation

2D A*는 차량을 크기가 없는 하나의 점으로 보므로, 원본 occupancy grid만 사용하면 벽과 주차선 바로 옆으로 경로가 생성될 수 있습니다. 이를 방지하기 위해 차량 반폭과 안전 마진을 고려한 obstacle inflation을 적용합니다.

현재 2D A* 테스트의 기본 inflation radius는 **7 cm**입니다. 원본 grid는 유지하고, 원형 타원 kernel로 장애물을 팽창한 별도 grid에서 A*를 수행합니다. Hybrid A*는 차량 크기를 inflation으로 대신하지 않고, 실제 회전된 rectangle footprint와 **1 cm** 추가 안전마진을 사용합니다.

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
- `output/hybrid_path_world_cm.csv`: Hybrid A*의 yaw·direction·steer를 포함한 제어용 pose 경로
- `output/hybrid_path_camera_bev.csv`: Hybrid pose를 Camera BEV pixel로 변환한 경로
- `output/hybrid_path_world_cm.json`: Hybrid pose 경로와 frame·planner metadata

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

Hybrid A*는 탐색 상태를 연속 `(x_cm, y_cm, yaw, direction)` pose로 확장합니다. wheelbase와 steering angle을 사용한 kinematic bicycle model로 3 cm motion primitive를 생성하므로, 2D A*처럼 움직임 방향이 즉시 45도씩 변하지 않습니다. 조향각 후보는 기본 `-30°`부터 `30°`까지 10° 간격이고, 전진과 후진을 모두 탐색합니다. 인접 primitive 사이의 조향 변화는 최대 10°로 제한하여 `-30°`에서 `+30°`로 즉시 바뀌는 경로를 생성하지 않습니다.

탐색 성능을 위한 3 cm motion primitive와 제어기에 전달하는 경로 간격을 분리합니다. 경로를 찾은 후에 각 primitive의 조향각과 `direction`을 유지한 채 bicycle model로 다시 적분하여 **0.5 cm 간격**의 고밀도 pose를 생성합니다. 따라서 단순 직선 보간과 달리 곡선을 따라 `x`, `y`, `yaw`가 점진적으로 변합니다. 간격은 `config/planner_config.yaml`의 `path_output_step_cm`으로 조정합니다.

차량 pose의 기준점은 **뒷바퀴 축 중심**입니다. 실차 측정값을 반영한 차량 `12×8 cm`, wheelbase `8 cm`, rear overhang `2 cm`로 회전된 직사각형이 덮는 모든 occupancy cell을 검사합니다. 차량 반폭 4 cm와 안전마진 1 cm를 합해 직선 벽에서 경로 중심선까지 명목상 5 cm를 확보합니다. motion primitive 중간도 0.5 cell 이하 간격으로 검사하므로, 회전 중 차체 모서리가 벽을 건너뛰는 경로를 차단합니다. 클릭한 start/goal이 기준점으로는 free여도 차체가 장애물을 침범하면 30 cm 반경 내의 가장 가까운 footprint-valid pose로 보정합니다.

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

## 다음 단계

- Reeds-Shepp analytic expansion 추가
- Hybrid 경로 smoothing과 속도 profile 생성
- 생성 경로를 PID/MPC 제어기에 전달1
