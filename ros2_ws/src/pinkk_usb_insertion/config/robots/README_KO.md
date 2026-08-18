# 로봇별 실행 프로필

공통 제어 파라미터는 `config/hybrid_runtime.yaml`에 두고 로봇마다 달라지는
보정값과 최소 덮어쓰기만 아래 폴더에서 관리한다.

| 프로필 | 기본 ROS Domain | Hand-eye | 용도 |
|---|---:|---|---|
| `robot_a` | 36 | `20260715_baseline_old` | 현재 로봇 A |
| `robot_b` | 38 | `20260807_153226_new_robot_20260807` | Git에서 받은 로봇 B |

각 폴더의 파일 역할:

- `camera_intrinsics.yaml`: 해당 카메라 내부행렬과 왜곡계수
- `handeye.yaml`: 해당 로봇 flange→camera 변환
- `runtime.yaml`: 카메라 장치, 관측 자세, 로봇별 제어 보정값 덮어쓰기

실행할 때 로봇 PC와 노트북에서 같은 프로필을 선택한다.

```bash
# Robot A
./scripts/run_robot_bridge.sh robot_a
./scripts/run_laptop_frozen_target_test.sh robot_a

# Robot B
./scripts/run_robot_bridge.sh robot_b
./scripts/run_laptop_frozen_target_test.sh robot_b
```

`ROS_DOMAIN_ID`를 직접 바꾸지 않아도 스크립트가 프로필에 맞춰 36/38을
선택한다. 특별한 진단에서만 `PINKK_ROS_DOMAIN_ID`로 덮어쓴다.

