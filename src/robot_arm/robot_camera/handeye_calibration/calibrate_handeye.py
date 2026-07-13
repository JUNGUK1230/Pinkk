"""저장된 sample로 5개 Hand-Eye 방법을 계산하고 검증한다."""

import argparse
from pathlib import Path

import numpy as np

from . import config
from .handeye_utils import calibrate_all_methods, result_score
from .io_utils import load_samples


def parse_args() -> argparse.Namespace:
    """계산할 sample 경로와 최종 저장 방법을 읽는다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=Path, default=config.SAMPLES_PATH)
    parser.add_argument("--method", help="최종 저장 방법. 생략하면 검증 점수 최저 방법")
    return parser.parse_args()


def main() -> None:
    """방법별 결과를 출력하고 선택 결과를 npy/npz로 저장한다."""
    args = parse_args()
    samples = load_samples(args.samples)
    count = len(samples["T_base_flange"])
    if count < config.MIN_CALIBRATION_SAMPLES:
        raise RuntimeError(f"sample {count}개: 최소 {config.MIN_CALIBRATION_SAMPLES}개 필요")
    results, failures = calibrate_all_methods(samples)
    print("METHOD      POS_MEAN_MM  POS_MAX_MM  ROT_MEAN_DEG  ROT_MAX_DEG")
    for result in results:
        print(f"{result.method:<11} {result.position_mean_mm:11.3f} "
              f"{result.position_max_mm:11.3f} {result.rotation_mean_deg:13.3f} "
              f"{result.rotation_max_deg:12.3f}")
        print(f"  보드 XYZ 평균 [m]: {result.xyz_mean_m}")
        print(f"  보드 XYZ 표준편차 [m]: {result.xyz_std_m}")
        print(f"  이동 벡터 [m]: {result.t_cam2flange.ravel()}")
        print(f"  이동 벡터 [mm]: {result.t_cam2flange.ravel() * 1000.0}")
        print(f"  T_flange_camera:\n{result.T_flange_camera}")
    for method, message in failures.items():
        print(f"{method} 계산 실패: {message}")
    if not results:
        raise RuntimeError("유효한 Hand-Eye 결과가 없습니다")
    recommended = min(results, key=result_score)
    selected = recommended
    if args.method:
        selected = next((item for item in results if item.method == args.method.upper()), None)
        if selected is None:
            raise ValueError(f"저장할 수 없는 방법: {args.method}")
    print(f"추천 방법: {recommended.method} | 위치 평균 "
          f"{recommended.position_mean_mm:.3f}mm, 회전 평균 "
          f"{recommended.rotation_mean_deg:.3f}deg")
    print(f"최종 저장 방법: {selected.method}")
    config.DATA_DIR.mkdir(parents=True, exist_ok=True)
    np.save(config.RESULT_MATRIX_PATH, selected.T_flange_camera)
    np.savez(
        config.RESULT_NPZ_PATH,
        R_cam2flange=selected.R_cam2flange,
        t_cam2flange=selected.t_cam2flange,
        T_flange_camera=selected.T_flange_camera,
        method=selected.method,
    )
    print(f"저장 완료: {config.RESULT_MATRIX_PATH}")
    print(f"저장 완료: {config.RESULT_NPZ_PATH}")


if __name__ == "__main__":
    main()
