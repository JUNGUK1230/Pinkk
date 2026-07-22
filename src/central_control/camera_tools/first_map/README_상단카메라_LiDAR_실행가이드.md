# First Map 파일 구성

## 실행 코드

| 파일 | 역할 |
|---|---|
| `capture_camera_bev.py` | 현재 USB 카메라 프레임을 왜곡 보정하고 1600×800 BEV로 변환해 `camera_bev.png`를 저장합니다. |
| `capture_bev_image.py` | YOLO 학습 데이터 수집용 BEV 영상만 녹화합니다. 원본 영상은 저장하지 않습니다. |
| `extract_bev_frames.py` | 녹화된 BEV 영상에서 CVAT용 프레임을 균등하게 추출합니다. |
| `register_bev_lidar_rigid.py` | Camera BEV 기준점과 LiDAR map 기준점으로 rigid affine 정합을 계산합니다. |
| `live_camera_lidar_registration.py` | 보정·BEV·LiDAR 정합 결과를 실시간으로 겹쳐 확인합니다. |
| `coordinate_transformer.py` | Camera BEV pixel, LiDAR map pixel, ROS map 좌표를 상호 변환합니다. |

## 보정·정합 데이터

| 파일 | 역할 |
|---|---|
| `camera_calibration.npz` | 카메라 intrinsic matrix와 distortion coefficient입니다. |
| `bev_homography.npz` | 원본 카메라 image를 1600×800 BEV로 변환하는 homography입니다. |
| `camera_to_lidar_rigid_registration.npz` | Camera BEV pixel에서 LiDAR map pixel로 가는 affine matrix와 정합 오차입니다. |

## 지도·이미지

| 파일 | 역할 |
|---|---|
| `camera_bev.png` | 카메라 없이 확인할 때 사용하는 저장된 Camera BEV 한 장입니다. |
| `my_test_map0710.png` | Occupancy grid 생성에 사용하는 LiDAR map 이미지입니다. |
| `my_test_map0710.yaml` | 지도 image 경로, resolution, origin 정보를 정의합니다. |

## 실행 중 생성되는 폴더

| 폴더 | 역할 |
|---|---|
| `bev_recordings/` | `capture_bev_image.py`가 만든 BEV 동영상을 저장합니다. |
| `bev_dataset/` | `extract_bev_frames.py`가 만든 CVAT용 이미지와 manifest를 저장합니다. |
