# Camera Tools 파일 구성

## `calibration/`

| 파일 | 역할 |
|---|---|
| `capture_checkerboard.py` | USB 카메라의 체커보드 보정 이미지를 촬영합니다. |
| `calibrate_camera.py` | 체커보드 이미지로 intrinsic matrix와 distortion coefficient를 계산합니다. |
| `camera_calibration.npz` | 계산된 카메라 내부 파라미터와 왜곡 계수를 저장합니다. |
| `__init__.py` | calibration Python 패키지를 정의합니다. |

## `bird_eye_view/`

| 파일 | 역할 |
|---|---|
| `bev_homography_test.py` | 카메라 기준점을 선택해 BEV homography를 생성합니다. |
| `bev_coordinate_test.py` | 생성된 BEV의 pixel과 실제 cm 좌표 관계를 확인합니다. |
| `camera_world_calibration.py` | Camera pixel과 실제 주차장 world 좌표 사이 homography를 계산합니다. |
| `live_metric_bev.py` | world homography가 적용된 실시간 metric BEV를 표시합니다. |
| `bev_homography.npz` | Camera image에서 BEV로 가는 homography와 출력 크기를 저장합니다. |
| `camera_to_world_homography.npz` | Camera image에서 metric world BEV로 가는 변환을 저장합니다. |
| `__init__.py` | bird-eye-view Python 패키지를 정의합니다. |

## `first_map/`

현재 사용하는 1600×800 Camera BEV, LiDAR 지도, affine 정합 데이터와 관련
도구를 보관합니다. 각 파일 역할은 폴더 내부 README에 정리되어 있습니다.
