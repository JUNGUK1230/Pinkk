# Hand-eye 영구 기록

이 폴더는 Hand-eye 수집 원본, 계산 결과, 자세 비교 결과와 현재 활성값을 모두
Git으로 추적합니다. 새 캘리브레이션을 기존 파일 위에 덮어쓰지 않습니다.

```text
data/
  runs/
    YYYYMMDD_HHMMSS_<label>/
      metadata.json          실행 환경, 샘플 수, 파일 hash
      samples.samples        Easy Handeye2 원본 샘플
      calibration.calib      Easy Handeye2 원본 결과
      T_flange_camera.npy    위 결과에서 생성한 4×4 행렬
  comparisons/
    YYYYMMDD_HHMMSS_<old>_vs_<new>/
      metadata.json
      measurements.csv
      measurements.summary.json
  active/
    manifest.json            현재 선택한 run
    calibration.calib
    T_flange_camera.npy
```

`runs/`와 `comparisons/`는 이력이며 수정하거나 재사용하지 않습니다. 같은 이름으로
다시 실행해도 시각 또는 순번이 붙은 새 폴더가 만들어집니다.

`active/`만 명시적인 `activate` 명령으로 바뀝니다. 이때 다음 호환 파일도 함께
동기화됩니다.

- `data/T_flange_camera.npy`
- `data/T_flange_camera_easy_handeye.npy`
- `ros2_ws/src/pinkk_usb_insertion/config/handeye.yaml`
- 설치된 `install_pinkk/.../pinkk_usb_insertion/config/handeye.yaml`(존재할 때)

명령:

```bash
bash scripts/calibration/laptop_handeye_data.sh list
bash scripts/calibration/laptop_handeye_data.sh show-active
bash scripts/calibration/laptop_handeye_data.sh activate 20260715_baseline_old
```

카메라 원본 이미지처럼 용량이 큰 데이터는 별도 저장 정책을 사용합니다. 이
폴더의 Hand-eye `.samples`, `.calib`, `.npy`, `.csv`, `.json`은 무시하지 않습니다.
