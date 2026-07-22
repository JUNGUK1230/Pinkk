# Legacy Backup 파일 구성

이 폴더는 2026-07-22 정리에서 활성 파이프라인과 분리한 파일을 원래
`central_control` 상대 경로대로 보관합니다.

## 초기 실행 구조

| 파일 | 역할 |
|---|---|
| `main.py` | 사용 전 전체 파이프라인 진입점이었던 초기 파일입니다. |
| `pipeline.py` | 기능이 연결되지 않은 초기 pipeline 뼈대입니다. |
| `scripts/run_project.sh` | 초기 pipeline 실행 shell입니다. |
| `scripts/run_yolo_seg.sh` | 이전 YOLO prototype 실행 shell입니다. |
| `tests/test_imports.py` | 초기 패키지 import smoke test입니다. |

## 카메라·YOLO 실험

| 파일 또는 폴더 | 역할 |
|---|---|
| `camera_tools/calibration/image_test1.py` | 이전 렌즈 보정 이미지 실험입니다. |
| `camera_tools/calibration/test1.py` | 이전 카메라 보정 테스트입니다. |
| `camera_tools/calibration/calibration_images/` | 기존 체커보드 원본 사진을 보관합니다. |
| `camera_tools/first_map/live_coordinate_test.py` | 이전 실시간 좌표 변환 확인 도구입니다. |
| `camera_tools/first_map/live_yolo_bev_map.py` | 현재 localization 이전의 YOLO·BEV·LiDAR 통합 prototype입니다. |
| `camera_tools/first_map/camera_lidar_rigid_overlay.png` | 이전 rigid registration 확인 이미지입니다. |
| `camera_tools/first_map/bev_recordings/` | 기존 YOLO 데이터 수집용 BEV 동영상입니다. |
| `camera_tools/first_map/bev_dataset/` | 기존 CVAT 이미지·manifest·압축본입니다. |

## 대체된 초기 모듈

| 폴더 또는 파일 | 역할 |
|---|---|
| `overhead_vision/map_registration/` | 구현되지 않은 LiDAR loader·occupancy·registration 뼈대입니다. |
| `overhead_vision/parking_detection/` | 초기 YOLO wrapper와 미구현 주차면 선택·goal 생성 뼈대입니다. |
| `overhead_vision/path_planning/astar.py` | 미구현 A* 모듈 뼈대입니다. |
| `overhead_vision/path_planning/obstacle_inflation.py` | 미구현 inflation 모듈 뼈대입니다. |
| `overhead_vision/path_planning/path_smoothing.py` | 미구현 smoothing 모듈 뼈대입니다. |

## 이전 경로계획 실험

| 파일 | 역할 |
|---|---|
| `path_planning/config/map_config.yaml` | 초기 2D A*·BEV 테스트에서 사용한 지도 경로 설정입니다. |
| `path_planning/src/astar_planner.py` | 이전 grid 기반 2D A* 구현입니다. |
| `path_planning/src/coordinate_transform.py` | 이전 2D A*의 grid·world cm·BEV 좌표 변환 구현입니다. |
| `path_planning/src/path_postprocess.py` | 이전 2D A* 경로의 RDP 단순화와 yaw 생성 구현입니다. |
| `path_planning/scripts/test_map_load.py` | 초기 occupancy map 로딩 테스트입니다. |
| `path_planning/scripts/test_astar.py` | 2D A*와 obstacle inflation 테스트입니다. |
| `path_planning/scripts/test_astar_overlay.py` | 2D A* rigid overlay 테스트입니다. |
| `path_planning/scripts/test_astar_on_camera_bev.py` | 2D A* Camera BEV 투영 테스트입니다. |
| `path_planning/scripts/click_astar_on_camera_bev.py` | Camera BEV 클릭 기반 2D A* 테스트입니다. |
| `path_planning/scripts/click_hybrid_astar_on_camera_bev.py` | Camera BEV 클릭 기반 Hybrid A* 테스트입니다. |
| `path_planning/scripts/test_hybrid_astar.py` | 초기 Hybrid A* 단위 테스트입니다. |
| `path_planning/scripts/test_hybrid_astar_analytic.py` | analytic expansion과 smoothing 테스트입니다. |
| `path_planning/scripts/test_reeds_shepp.py` | Reeds–Shepp 수학 경로 테스트입니다. |
| `path_planning/scripts/test_reeds_shepp_collision.py` | Reeds–Shepp footprint 충돌 테스트입니다. |
| `path_planning/scripts/overlay_parking_slots.py` | Camera BEV에 주차면 polygon을 표시하던 도구입니다. |
| `path_planning/output/` | 위 실험에서 생성된 이미지, CSV, JSON 결과를 보관합니다. |

BEV 동영상과 CVAT 데이터는 대용량 생성 파일이므로 Git에는 포함하지 않고 로컬
백업으로만 유지합니다.
