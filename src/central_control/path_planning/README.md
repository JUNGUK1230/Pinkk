# 고정 경로 계획

현재 실시간 주행은 고정된 주차장 구조에 맞춘 사전 생성 경로를 사용합니다.
YOLO로 검출한 차량이 `START` 또는 어느 주차면에 있는지 판별하고, 현재
section과 목표 section에 대응하는 CSV 전체를 선택합니다. 차량이 항상 출발지
또는 주차면에서 출발하므로 도로 중간 경로를 잘라 붙이지 않습니다.

이 방식은 매번 A*/Hybrid A*를 탐색하는 것보다 결과가 일정하고 디버깅하기
쉽습니다. 주차장 구조나 주행 순서가 바뀌면
`config/fixed_mission_routes.yaml`과 고정 경로 CSV를 다시 생성해야 합니다.

## 운영 파일

| 파일 | 역할 |
|---|---|
| `config/fixed_mission_routes.yaml` | 고정 yaw, 도로 중심선, endpoint pose와 허용 이동 관계를 정의합니다. 모든 좌표는 `lidar_map_cm` 기준입니다. |
| `src/fixed_route_selector.py` | 현재 차량 중심 pose를 `START`/주차면으로 분류하고 해당 CSV 전체를 불러옵니다. |
| `scripts/test_fixed_mission_routes.py` | 허용된 32개 경로를 생성하거나 충돌 여부를 검사합니다. |
| `scripts/test_fixed_route_selector.py` | 현재 section 판별과 CSV 선택을 회귀 검사합니다. |
| `scripts/test_fixed_live_route_bridge.py` | YOLO scene의 현재 위치·목표가 올바른 고정 경로로 연결되는지 검사합니다. |
| `output/fixed_route_manifest.csv` | source/target별 CSV 이름과 point 수를 정리합니다. |
| `output/fixed_route_*_to_*.csv` | 차량 제어에 사용하는 고정 경로입니다. |

각 CSV 필드는 `index, x_cm, y_cm, yaw_rad, direction`입니다.
`direction`은 전진 `1`, 후진 `-1`이며 속도와 조향각은 포함하지 않습니다.
주행 제어기가 경로와 direction을 이용해 속도·조향을 결정합니다.

## 허용 경로

- `START` → `P8`~`P5`, `C1`, `C2`
- `P8`~`P5` → `C1`, `C2`
- `C1`, `C2` → `P1`~`P4`
- `P1`~`P4` → `EXIT`

같은 단계의 주차면 사이 이동(`P8`→`P7`, `C1`→`C2`, `P1`→`P2` 등)은
생성·표시·토픽 발행 대상이 아닙니다.

슬롯에서 출차할 때는 해당 주차 진입 경로를 반대로 따라 전진합니다. 목표
슬롯에서는 도로 중앙 staging pose에서 한 번의 연속 후진 maneuver로
진입합니다.

## 생성과 검사

공용 도로의 코너는 반경 16 cm의 원호로 연결하고, 주차 진입·후진 구간은
최소 회전 반경 14 cm를 만족하도록 생성합니다. 전체 경로는 0.5 cm 간격으로
샘플링합니다. `--check-all`은 장애물 충돌과 경로 곡률을 함께 검사하여 현재
차량 설정의 최대 조향각 30도 이내인지 확인합니다.

```bash
cd ~/PINKK/src/central_control/path_planning

# 허용된 32개 CSV와 manifest 다시 생성
python3 scripts/test_fixed_mission_routes.py --generate-all

# 파일을 덮어쓰지 않고 모든 경로 검사
python3 scripts/test_fixed_mission_routes.py --check-all

# 개별 경로 생성
python3 scripts/test_fixed_mission_routes.py --source START --target C2

# section 선택 및 실시간 연결 회귀 검사
python3 scripts/test_fixed_route_selector.py
python3 scripts/test_fixed_live_route_bridge.py
```

PNG 경로 이미지는 운영 입력이 아니므로 생성하지 않습니다.

## 실시간 연결

`overhead_vision/localization/live_localization.py`가 다음 순서로 동작합니다.

1. YOLO/ByteTrack으로 ego 차량의 중심을 검출합니다.
2. Camera–LiDAR 정합으로 차량 중심 위치를 `lidar_map_cm`으로 변환합니다.
3. 차량 중심이 포함된 주차면 또는 `START`를 현재 section으로 사용합니다.
4. 현재 section과 배정된 목표 section으로 고정 CSV를 선택합니다.
5. 선택한 경로를 파일 중계 없이 ROS 2 토픽으로 발행합니다.

차량 앞쪽을 클릭하는 heading 입력은 사용하지 않습니다. 카메라 마스크 장축의
앞·뒤 판별에는 `fixed_mission_routes.yaml`의 endpoint yaw를 사용합니다.
차량이 `TRANSIT` 상태이면 새 경로를 선택하지 않습니다.

실시간 확정 pose는 `/pinkk/vehicle_N/localization_pose`로 별도 발행하며 고정 CSV 앞에
삽입하지 않습니다. 검출 오차가 있는 pose를 첫 경로점으로 붙이면 START anchor
사이에 인공적인 급회전 구간이 생길 수 있기 때문입니다.

새 경로는 선택 즉시 발행하고, 이후
`config/yolo/realtime_localization.yaml`의
`route_republish_period_sec` 주기로 동일 경로를 반복 발행합니다. 기본값은
1초입니다. 목표 변경이나 경로 무효화 시 이전 경로 재발행은 즉시 중단됩니다.

발행 토픽:

- `/pinkk/vehicle_pose`: `geometry_msgs/PoseStamped`, 현재 차량 중심 pose
- `/pinkk/planned_path`: `nav_msgs/Path`, m 단위, `lidar_map` frame
- `/pinkk/planned_trajectory`: `std_msgs/Float64MultiArray`
  (`x_m, y_m, yaw_rad, direction`)

## 레거시·오프라인 진단

`hybrid_astar_planner.py`, `t_parking_planner.py`, `trajectory_profile.py`,
`trajectory_validator.py`, `plan_from_live_vision.py`는 경로 형상 실험과
회귀 진단을 위해 남겨 둔 코드입니다. 현재 실시간 경로 선택·발행 흐름에서는
Hybrid A* 탐색, velocity profile, steering profile을 사용하지 않습니다.

구현 변경 기록은 `DEVELOPMENT_MEMO.md`, 외부 코드 라이선스는
`THIRD_PARTY_NOTICES.md`에서 확인할 수 있습니다.
