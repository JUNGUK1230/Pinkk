"""정지한 로봇의 T_base_flange와 T_camera_charuco sample을 수집한다."""

import argparse
from pathlib import Path
import time

import cv2
import numpy as np

from ..config import settings as config
from ..core.charuco import create_board_and_detector, estimate_charuco_pose, load_intrinsics
from ..core.io import camera_arg, open_camera, save_samples, verify_resolution
from ..core.robot_adapter import create_robot, validate_robot_frames
from ..core.transforms import pose_difference, robot_coords_to_T_base_flange


def parse_args() -> argparse.Namespace:
    """sample 수집 옵션을 읽는다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=camera_arg, default=config.CAMERA_DEVICE)
    parser.add_argument("--samples", type=Path, default=config.SAMPLES_PATH)
    parser.add_argument("--overwrite", action="store_true", help="기존 sample 파일 덮어쓰기")
    return parser.parse_args()


def print_guide() -> None:
    """보드 고정과 다양한 회전 자세의 중요성을 출력한다."""
    print("\nChArUco 보드는 수집이 끝날 때까지 절대 움직이지 마세요.")
    print("로봇을 완전히 정지하고 진동이 가라앉은 뒤 S 또는 Space를 누르세요.")
    print("정면, 좌/우, 앞/뒤, 대각선 기울임 등 회전 자세를 다양하게 만드세요.")
    print(f"목표 {config.TARGET_SAMPLE_COUNT}개, 권장 15~30개입니다.\n")


def draw_overlay(frame: np.ndarray, pose, sample_count: int) -> None:
    """OpenCV 영상에 영문 상태 문구와 검출 코너를 표시한다."""
    status = "DETECTED" if pose.success else "NOT DETECTED"
    error = f"{pose.reprojection_error:.3f}px" if pose.success else "--"
    lines = [
        f"Charuco: {status} | corners: {pose.detected_corner_count}",
        f"reprojection: {error} | samples: {sample_count}",
        "S/Space: save | Q/ESC: quit",
    ]
    for index, line in enumerate(lines):
        cv2.putText(frame, line, (15, 30 + index * 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 255, 0), 2, cv2.LINE_AA)
    if pose.corners is not None and pose.ids is not None:
        cv2.aruco.drawDetectedCornersCharuco(frame, pose.corners, pose.ids)


def main() -> None:
    """로봇 PC 로컬 카메라와 mc 객체로 sample 수집 loop를 실행한다."""
    args = parse_args()
    if args.samples.exists() and not args.overwrite:
        raise FileExistsError(f"기존 sample이 있습니다: {args.samples}; 새 수집은 --overwrite 사용")
    if not config.ROBOT_EULER_CONVENTION_VERIFIED:
        raise RuntimeError("ROBOT EULER CONVENTION NOT VERIFIED")
    camera_matrix, dist_coeffs, expected_size = load_intrinsics(config.INTRINSICS_PATH)
    board, detector = create_board_and_detector()
    robot = create_robot()
    validate_robot_frames(robot)
    capture = open_camera(args.camera, config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
    verify_resolution(capture, expected_size)
    print_guide()
    samples: list[dict[str, object]] = []
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                print("카메라 frame 읽기 실패")
                break
            pose = estimate_charuco_pose(frame, camera_matrix, dist_coeffs, board, detector)
            draw_overlay(frame, pose, len(samples))
            cv2.imshow("handeye_sample_collection", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key not in (ord("s"), ord("S"), 32):
                continue
            print("로봇 정지 상태를 전제로 안정화 대기 후 sample을 읽습니다...")
            time.sleep(config.CAPTURE_SETTLE_SECONDS)
            ok, fresh_frame = capture.read()
            if not ok:
                print("저장 거부: frame 읽기 실패")
                continue
            fresh = estimate_charuco_pose(
                fresh_frame, camera_matrix, dist_coeffs, board, detector
            )
            if not fresh.success or fresh.reprojection_error > config.MAX_REPROJECTION_ERROR_PX:
                print(f"저장 거부: ChArUco pose/재투영 오차 {fresh.reprojection_error:.3f}px")
                continue
            coords = robot.get_coords()
            if coords is None:
                print("저장 거부: get_coords() 실패")
                continue
            try:
                base_flange, rotation, translation = robot_coords_to_T_base_flange(coords)
            except (TypeError, ValueError, RuntimeError) as error:
                print(f"저장 거부: 잘못된 로봇 pose: {error}")
                continue
            if samples:
                distance, angle = pose_difference(samples[-1]["T_base_flange"], base_flange)
                if (distance < config.MIN_TRANSLATION_DIFFERENCE_M
                        and angle < config.MIN_ROTATION_DIFFERENCE_DEG):
                    print(f"경고: 직전 pose와 유사함 ({distance*1000:.1f}mm, {angle:.1f}deg)")
            samples.append({
                "sample_index": len(samples),
                "robot_coords_raw": np.asarray(coords, dtype=float),
                "T_base_flange": base_flange,
                "R_gripper2base": rotation,
                "t_gripper2base": translation,
                "T_camera_charuco": fresh.T_camera_charuco,
                "R_target2cam": fresh.R_target2cam,
                "t_target2cam": fresh.t_target2cam,
                "reprojection_error": fresh.reprojection_error,
                "detected_corner_count": fresh.detected_corner_count,
                "timestamp": time.time(),
            })
            save_samples(args.samples, samples)
            print(f"sample {len(samples)}개 저장 완료: {args.samples}")
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
