# Pinkk USB 삽입 인터페이스

YOLO keypoint 검출 노드와 USB pose 계산 노드 사이의 ROS 2 메시지를 정의합니다.

```text
YOLO 노드
→ UsbPortDetectionArray
→ port_pose_node
→ UsbPortObservation
```

메시지에 영상의 `header`, 검출 ID, 객체 신뢰도, 네 keypoint의 좌표·신뢰도,
bounding box와 원본 해상도를 함께 넣어 서로 다른 프레임의 값이 섞이지 않게 합니다.

keypoint index는 다음 물리적 순서를 고정해서 사용합니다.

```text
0 → 1: USB 포트의 긴 변 11.5 mm
1 → 2: 인접한 짧은 변 4.5 mm
2 → 3 → 0: 같은 방향으로 나머지 둘레
```

YOLO 데이터셋 라벨 순서와 `port_pose_node`의 3D 모델점 순서가 반드시 같아야
합니다.
