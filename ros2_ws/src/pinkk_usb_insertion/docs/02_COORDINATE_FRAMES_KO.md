# 좌표계와 행렬 방향

USB 삽입 오차의 상당 부분은 행렬 값 자체보다 행렬 방향을 반대로 사용해서
발생합니다. 이 문서의 표기를 코드와 로그에 동일하게 사용합니다.

## 1. 좌표계

```text
g_base
└── joint6_flange
    ├── camera_optical_frame
    │   └── usb_port
    └── plug_tip
```

| 프레임 | 의미 |
|---|---|
| `g_base` | 로봇 기준 좌표계 |
| `joint6_flange` | URDF의 마지막 flange 프레임 |
| `camera_optical_frame` | OpenCV와 동일한 optical frame |
| `usb_port` | 포트 입구 중심과 포트 방향 |
| `plug_tip` | 실제 USB 플러그 끝단 기준 |

카메라 optical frame은 X가 영상 오른쪽, Y가 영상 아래, Z가 카메라 정면입니다.

## 2. 행렬 표기

`T_a_b`는 **a 좌표계에서 본 b 좌표계 자세**입니다. b 좌표의 점을 a 좌표로
바꾸는 행렬이기도 합니다.

```text
p_a = T_a_b × p_b
```

포트의 base 자세는 다음과 같습니다.

```text
T_base_port
= T_base_flange
× T_flange_camera
× T_camera_port
```

현재 plug tip은 다음과 같습니다.

```text
T_base_plug
= T_base_flange
× T_flange_plug
```

## 3. 접근 자세 계산

포트 좌표계의 삽입축을 +Z로 정의하면 포트 전방 standoff는 -Z입니다.

```text
T_port_approach.translation = [0, 0, -standoff]
```

목표 plug tip 자세는:

```text
T_base_plug_goal = T_base_port × T_port_approach
```

실제로 명령해야 하는 flange 자세는:

```text
T_base_flange_goal
= T_base_plug_goal
× inverse(T_flange_plug)
```

TCP가 없는 상태에서 `T_flange_plug=identity`를 넣으면 계산 시험은 가능하지만 실제
삽입 목표로 사용하면 안 됩니다.

## 4. 설정 파일 대응

| 행렬 | 설정 파일 |
|---|---|
| K, D | `camera_intrinsics.yaml` |
| `T_flange_camera` | `handeye.yaml` |
| `T_flange_plug` | `tool_transform.yaml` |
| standoff 및 제한 | `insertion_control.yaml` |

## 5. 검증 체크리스트

- 카메라가 포트에 가까워질수록 `T_camera_port.z`가 감소하는가?
- 포트가 영상 오른쪽으로 가면 카메라 X가 증가하는가?
- `T_base_port`가 로봇 자세 변화 후에도 같은 실제 위치를 나타내는가?
- standoff를 늘리면 목표가 포트에서 멀어지는가?
- `T_flange_plug`를 적용했을 때 flange가 아니라 plug tip이 목표에 놓이는가?

