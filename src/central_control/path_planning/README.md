# Path Planning

Python 기반 자율주행 주차 경로계획 모듈의 기본 구현입니다. 현재 단계는 Hybrid A* 구현 전의 **Occupancy Grid + 2D A*** 검증 단계로, LiDAR 지도를 읽고 좌표를 변환하며 장애물을 피해 최단 경로를 생성하는 흐름을 확인합니다.

## 구성

- `config/map_config.yaml`: 지도, BEV, LiDAR 및 정합 파일 설정
- `config/vehicle_config.yaml`: 차량 제원과 조향 제약
- `config/planner_config.yaml`: A* 및 향후 Hybrid A* 파라미터
- `src/occupancy_grid.py`: ROS map_server 형식 PGM/YAML 로더
- `src/coordinate_transform.py`: BEV pixel, world cm, grid 좌표 변환
- `src/astar_planner.py`: 4/8방향 2D A* 탐색
- `src/visualization.py`: 지도와 경로의 OpenCV 시각화
- `scripts/`: 지도 로딩 및 A* 실행 스크립트
- `output/`: 실행 결과 이미지

좌표는 A*와 occupancy grid에서 `(x, y) = (column, row)`를 사용하며 실제 배열 접근은 `grid[y, x]`입니다. BEV 원점은 좌상단이고 y가 아래로 증가하지만, world cm 원점은 좌하단이고 y가 위로 증가합니다.

`OccupancyGridMap`은 기본적으로 `block_outside_area=True`를 사용합니다. 외벽의 작은 끊김을 morphology close로 막은 임시 mask에서 테두리와 연결된 자유 공간을 찾고, 그 외부 영역을 최종 grid의 장애물로 처리합니다. morphology로 두께워진 임시 장애물은 최종 grid에 복사하지 않습니다.

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
```

생성되는 파일은 다음과 같습니다.

- `output/debug_occupancy_grid.png`: 흰색 자유 공간과 검은색 장애물로 표현한 occupancy grid
- `output/astar_result.png`: 경로(빨강), 시작점(초록), 목표점(파랑)을 표시한 A* 결과

`test_astar.py`는 기본 시작점 `(20, 20)`과 목표점 `(200, 180)`을 사용합니다. 점이 장애물이면 반경 30셀 안에서 가장 가까운 자유 셀을 찾고, 목표가 지도 밖이면 지도 크기에 맞춰 안쪽으로 보정합니다. 연결 가능한 경로가 없으면 이미지 대신 명확한 실패 메시지를 출력합니다.

## 다음 단계

- Hybrid A* 구현
- 상태에 차량 yaw 추가
- 전진/후진 direction 추가
- 차량 footprint collision check 추가
- 생성 경로를 PID/MPC 제어기에 전달
