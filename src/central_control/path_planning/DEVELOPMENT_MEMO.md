# 경로계획 중간 메모

## 2026-07-21: 좁은 주차면 충돌 조건 완화

- Hybrid A* 차량 크기: 길이 12cm, 폭 10cm, rear-axle 기준
- 추가 obstacle inflation: 2cm에서 0cm로 변경
- 실제 회전 차량 footprint와 원본 검은색 벽의 충돌 검사는 유지
- 고정 12cm 직선 후진 approach 조건 제거
- 고정거리 관련 설정과 live pipeline 실행 코드도 완전히 제거
- 최종 목표 pose까지 Hybrid A*가 전진·후진 조합을 직접 탐색
- 자동 주차의 마지막 이동 방향은 후진(`-1`)으로 유지
- 주차칸 이름에 따른 분기 없이 모든 칸에 같은 차량·지도·기구학 조건 적용

회귀 결과:

- C1: 성공, 약 0.56초
- P6: 성공, 약 0.16초
- P10: 성공, 약 0.21초
- P8/P9: 차체 충돌 즉시 거절은 해소됐지만 짧은 후보 timeout 안에서 조향 스무딩 경로 미확정

다음 작업은 P8/P9에서 생성된 Reeds–Shepp 후보가 조향 제한을 넘는 원인을 확인하고, 안전 충돌 검사를 유지한 채 smoothing 또는 goal heading을 조정하는 것이다.

## 2026-07-21: 자동 계획 시간 제한 해제

- live 자동 계획 전체 시간 제한: 30초에서 무제한(`0`)으로 변경
- goal 후보별 시간 제한: 5초에서 무제한(`0`)으로 변경
- 탐색 진행 로그는 1,000 expanded node마다 계속 출력
- 메모리·상태공간 폭증 보호용 `max_expanded_nodes: 50000`은 유지

## 2026-07-22: 실시간 목표 헤딩 조건 제거

- live planner는 primary/alternative rear-axle goal의 중점인 주차면 중심만 목표로 사용
- 목표 yaw 일치 조건과 heading 축 기준 주변 goal 반복 탐색을 제거
- Hybrid A* 내부의 시작 yaw, 조향, 차체 footprint 충돌 검사는 유지
- 마지막 이동 방향은 후진(`direction=-1`)으로 유지
- P5 heading-free 단일 목표 탐색은 50,000 node까지 확장했지만 해를 찾지 못했다. 목표 헤딩 제거만으로 전역 Hybrid A* 연산량 문제가 해결되지는 않는다.

## 2026-07-22: 공통 후진주차 입구 규칙 적용

- heading-free 목표 중심 방식은 후진 Reeds-Shepp 연결을 사용할 수 없어 자동 live planner에서 대체
- 각 주차칸 polygon의 짧은 두 변을 입구 후보로 생성
- occupancy map에서 바깥쪽 통로 여유가 더 큰 변을 입구로 자동 선택
- 차량 앞은 입구를 향하고 rear axle은 주차칸 안쪽에 배치
- 주차칸 이름별 조건문 없이 P1~P10, C1~C2 모두 footprint-valid goal을 생성하는 회귀 확인
- P5는 하나의 입구 heading으로 축소됐지만 전역 Hybrid A*가 8초/약 3만 node 안에 해를 찾지 못했다. 속도 문제는 별도의 전역 탐색 구조 개선이 필요하다.

## 2026-07-22: 입구 기준 첫 주차칸 자동 배정

- `config/map/parking_points.json`에 입구·출구의 BEV 좌표를 분리해 저장했다.
- 현재 활성 단계 `entry_to_parking`은 입구 기준으로 P6~P10의 빈 칸을 먼 순서로 정렬하고, 첫 번째 후보만 Hybrid A*에 전달한다.
- 점유된 칸은 후보에서 제외하므로 가장 먼 칸이 차 있으면 다음 거리 순위 칸으로 자동 변경된다.
- 후보 목록은 scene JSON에 함께 저장해 planner와 화면에서 선택 근거를 확인할 수 있다.
- 다음 단계는 `P6~P10 → C1/C2 → P1~P5 → exit` 상태 전환을 별도 episode controller로 구현하는 것이다. 아직 충전소·출구 단계는 선택하지 않는다.

## 2026-07-22: T자 후면주차 planner 분리

- 주차칸 입구 방향을 기준으로 15~30cm 앞 통로에 staging pose를 만든다.
- 첫 Hybrid A*는 현재 pose에서 staging pose까지 접근하고, 두 번째 Hybrid A*는 staging에서 final pose까지 T자 전진·후진 maneuver만 만든다.
- 두 구간 사이에는 연속 stop point를 넣어 정지 상태에서 조향을 바꿀 수 있게 했다.
- staging 목표의 yaw 허용오차도 5°로 제한했다. 이전 15° 허용오차에서는 접근 경로와 maneuver 사이에 yaw jump가 생길 수 있었다.
- P6 기준 회귀에서는 접근 약 200 node와 maneuver 약 80 node로, 마지막 후진 10cm 이상 조건을 통과했다.
- 현장 start에서 통로 접근이 3초 안에 실패하면 후보를 최대 4개까지만 검사하고 fail-closed 한다. 다음 개선은 2D 통로 guide를 Hybrid 접근 탐색의 heuristic으로 직접 사용하는 것이다.

## 2026-07-22: 검증된 trajectory ROS 2 발행

- `overhead_vision/path_planning/path_publisher.py`가 `live_hybrid_path_world_cm.json`을 감시해 ROS 2 토픽으로 발행한다.
- `/pinkk/planned_path`는 m 단위 `nav_msgs/Path`, `/pinkk/planned_trajectory`는 전후진·속도·조향·정지 정보를 포함한 7열 `Float64MultiArray`다.
- planner가 validator를 통과해 저장한 `planner: hybrid_astar` trajectory만 발행한다. `control_ready: false` 또는 validation 정보가 없는 파일은 fail-closed로 거부한다.
- 아직 제어기 subscriber는 연결하지 않았으며, 현재 단계는 토픽 규약·단위·발행 확인이다.

## 2026-07-22: 실시간 차량 pose ROS 2 발행

- `/pinkk/vehicle_pose`에 상단 카메라 localization의 rear-axle `(x_m, y_m, yaw)`를 `PoseStamped`로 발행한다.
- `live_vision_scene.json`의 새 frame만 감시하며, planning-ready가 아니거나 0.5초보다 오래된 scene은 발행하지 않는다.
- 경로 토픽과 현재 pose 토픽 모두 `lidar_map` frame과 m 단위를 사용해 PP/PI 제어기가 직접 비교할 수 있게 맞췄다.

## 2026-07-22: Hybrid A* 조향각 해상도 5° 적용

- 조향 물리 한계는 ±30°로 유지하고, 탐색 후보를 기존 10° 간격 7개에서 5° 간격 13개로 늘렸다.
- T자 주차 maneuver에서 더 완만한 곡선 후보를 찾을 수 있다.
- 3cm motion primitive당 최대 조향 변화 10° 제한은 유지해 조향각 순간 점프를 막는다.
- 후보 수 증가로 탐색량은 커질 수 있으므로 T자 staging 분리와 함께 실행 시간을 계속 확인한다.

## 2026-07-22: central_control 활성 파이프라인 기준 파일 정리

- 실시간 YOLO localization, T자 후진주차 Hybrid A*, trajectory validator,
  ROS 2 경로·차량 pose publisher와 관련 회귀 테스트는 현재 위치에 유지했다.
- 상단 카메라 렌즈 보정, BEV homography, Camera–LiDAR 정합 코드와 NPZ,
  지도 및 모델 파일도 현재 위치에 유지했다.
- 예전 클릭식 2D/Hybrid A* 테스트, 초기 YOLO prototype, 미구현 뼈대 모듈,
  `live_*`가 아닌 과거 결과 이미지·좌표 파일은 삭제하지 않고
  `central_control/backup/legacy_20260722`로 이동했다.
- 실시간 planner가 예전 `test_astar.py`에서 가져오던 지도 이미지 선택 함수를
  `plan_from_live_vision.py` 내부로 옮겨 legacy test에 대한 실행 의존성을 제거했다.
- 이후 새 기능은 활성 파이프라인에 추가하고, 일회성 실험 파일은 같은 백업 정책으로
  분리해 운영 코드와 섞이지 않도록 한다.

## 2026-07-22: README 역할 분리

- 작업 범위의 최상위인 `central_control/README.md`에는 환경 설치, 실제 실행
  순서, 중앙제어 알고리즘의 짧은 흐름만 남겼다.
- `central_control` 내부 하위 README는 실행 이력과 장문의 알고리즘 설명을
  제거하고, 해당 폴더에 실제로 존재하는 파일·하위 폴더 역할로 통일했다.
- 이전 `~/project` 경로, 삭제된 실행 shell, 클릭 기반 테스트, 과거 P5·2cm
  inflation 설명처럼 현재 코드와 맞지 않는 안내를 제거했다.
- 실험 판단과 수치 변경 이력은 README가 아니라 이 `DEVELOPMENT_MEMO.md`에서
  계속 관리한다.
- 저장소 루트, `robot_arm`, `vehicle_control`, ROS package setup은 담당 범위가
  아니므로 수정 전 상태로 복원했다.

## 2026-07-22: path_planning 잔여 파일 정리

- 활성 코드에서 참조하지 않는 `astar_planner.py`, `coordinate_transform.py`,
  `path_postprocess.py`, `map_config.yaml`은 과거 2D A* 테스트 복구 가능성을 위해
  `central_control/backup/legacy_20260722/path_planning`으로 이동했다.
- `src/__init__.py`에서는 위 legacy API export를 제거하고 현재 Hybrid A* 관련
  API만 노출하도록 정리했다.
- Python이 언제든 다시 생성할 수 있는 `__pycache__` 34개는 백업하지 않고
  삭제했다.
- 활성 경로계획 폴더는 cache 제외 30개에서 26개로 줄었다.

## 2026-07-22: 차량 PID 제어기 후보 추가 및 격리 검증

- 전달받은 `pid_path_follower_smooth_topic.py`와
  `pid_path_follower_smooth_parking_complete_topic.py`를 `src/vehicle_control`에
  복사했다.
- 두 파일 모두 Python 문법 검사와 ROS 2 노드 생성을 통과했다.
- 실제 차량과 분리된 ROS domain 232에서 합성 `nav_msgs/Path`를
  적용했고, 전진 구간은 양수, 후진 구간은 음수 `linear.x`를
  생성하는 것을 확인했다. 테스트 중 `/cmd_vel` 발행 함수는 캡처로
  대체해 실제 모터 명령이 나가지 않게 했다.
- 두 파일의 중앙 `/pinkk/planned_path` 처리 방식은 같다. 두 번째
  파일의 차이는 중앙 경로 모드가 아닌 내장 waypoint 주차 시퀀스에
  있다.
- 현재 제어기는 `/pinkk/planned_trajectory`를 구독하지 않아 planner가
  계산한 direction, 조향각, 목표 속도, 정지 표시를 무시한다. 후진은
  `Path` 이동 벡터와 pose yaw의 내적으로 다시 추정한다.
- 현재 차량 자세는 `/pinkk/vehicle_pose`가 아닌 `/odom`에서 받는다.
  실차 연동 전에 trajectory 메타데이터와 pose frame을 제어기에 직접
  연결하는 보완이 필요하다.
- 루트 `.gitignore`가 `src/vehicle_control/` 전체를 제외하고 있어 두 Python
  파일은 현재 로컬 복사본이며 일반 `git add`로는 커밋되지 않는다.
  팀 소유 정책을 확인한 뒤 예외 규칙 또는 명시적 추적을 선택해야 한다.

## 2026-07-22: 남은 PID 제어기 경로 토픽 수신 수정

- 남은 `pid_path_follower_smooth_topic.py` 하나를 기준으로 ROS 2
  publish/subscribe를 다시 검증했다.
- 중앙 발행기는 `/pinkk/planned_path`를 `RELIABLE + TRANSIENT_LOCAL`로
  발행하지만 제어기는 기본 `VOLATILE`로 구독해, 발행기보다 나중에
  실행된 제어기가 마지막 경로를 받지 못하는 문제를 재현했다.
- 경로 subscriber의 QoS를 발행기와 같게 맞춰 late join 제어기도
  보존된 경로를 즉시 받도록 수정했다.
- 각 waypoint의 전후진을 다음 edge로 판정하던 로직은 gear cusp
  정지점을 한 점 일찍 후진으로 해석했다. Hybrid trajectory 규약에
  맞게 이전 점에서 현재 목표점으로 들어오는 edge를 기준으로 수정했다.
- 격리 ROS domain에서 `경로 선발행 → 제어기 late join → odom 수신`
  순서를 테스트했다. 보존 경로 수신, odom 후 활성화, 전진
  `+0.0625 m/s`, 후진 `-0.0625 m/s`를 모두 확인했다.
- 사용자 요청에 따라 이 작업 내용은 README에 추가하지 않았다.
