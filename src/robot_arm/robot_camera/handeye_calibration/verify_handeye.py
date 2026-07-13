"""저장한 T_flange_camera로 고정 보드의 base 좌표를 실시간 검증한다."""

import argparse
from pathlib import Path

import cv2
import numpy as np

from . import config
from .charuco_utils import create_board_and_detector, estimate_charuco_pose, load_intrinsics
from .io_utils import camera_arg, open_camera, verify_resolution
from .robot_adapter import create_robot, validate_robot_frames
from .transform_utils import robot_coords_to_T_base_flange, validate_transform


def parse_args() -> argparse.Namespace:
    """카메라와 결과행렬 경로를 읽는다."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=camera_arg, default=config.CAMERA_DEVICE)
    parser.add_argument("--result", type=Path, default=config.RESULT_MATRIX_PATH)
    return parser.parse_args()


def main() -> None:
    """로봇 pose를 바꿔도 T_base_charuco가 일정한지 화면에 표시한다."""
    args = parse_args()
    flange_camera = np.asarray(np.load(args.result, allow_pickle=False), dtype=float)
    validate_transform(flange_camera)
    camera_matrix, dist_coeffs, expected_size = load_intrinsics(config.INTRINSICS_PATH)
    board, detector = create_board_and_detector()
    robot = create_robot()
    validate_robot_frames(robot)
    capture = open_camera(args.camera, config.CAMERA_WIDTH, config.CAMERA_HEIGHT)
    verify_resolution(capture, expected_size)
    reference: np.ndarray | None = None
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            pose = estimate_charuco_pose(frame, camera_matrix, dist_coeffs, board, detector)
            current = None
            lines = ["Board not detected", "Q/ESC: quit | R: reset reference"]
            if pose.success:
                try:
                    base_flange, _, _ = robot_coords_to_T_base_flange(robot.get_coords())
                    base_charuco = base_flange @ flange_camera @ pose.T_camera_charuco
                    current = base_charuco[:3, 3]
                    if reference is None:
                        reference = current.copy()
                    deviation = float(np.linalg.norm(current - reference) * 1000.0)
                    lines = [
                        f"Board Base X [mm]: {current[0]*1000:.2f}",
                        f"Board Base Y [mm]: {current[1]*1000:.2f}",
                        f"Board Base Z [mm]: {current[2]*1000:.2f}",
                        f"Position deviation [mm]: {deviation:.2f}",
                        f"Reprojection error [px]: {pose.reprojection_error:.3f}",
                        "Q/ESC: quit | R: reset reference",
                    ]
                except (TypeError, ValueError, RuntimeError) as error:
                    lines = [f"Robot pose invalid: {error}"]
            for index, line in enumerate(lines):
                cv2.putText(frame, line, (15, 30 + index * 27), cv2.FONT_HERSHEY_SIMPLEX,
                            0.62, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("handeye_verification", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("r"), ord("R")) and current is not None:
                reference = current.copy()
    finally:
        capture.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
