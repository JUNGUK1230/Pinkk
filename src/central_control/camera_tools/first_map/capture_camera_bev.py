"""Capture and save a calibrated 1600x800 Camera BEV image.

Run:
    cd ~/PINKK/src/central_control/camera_tools/first_map
    python3 capture_camera_bev.py

Keys:
    s: save the current BEV as camera_bev.png
    q or ESC: quit
"""

from pathlib import Path
from typing import Sequence

import cv2
import numpy as np

# Camera settings are kept at the top so another USB device can be selected easily.
CAMERA_ID = 2
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
BEV_WIDTH = 1600
BEV_HEIGHT = 800

SCRIPT_DIR = Path(__file__).resolve().parent
CALIBRATION_PATH = SCRIPT_DIR / "camera_calibration.npz"
HOMOGRAPHY_PATH = SCRIPT_DIR / "bev_homography.npz"
OUTPUT_PATH = SCRIPT_DIR / "camera_bev.png"

CAMERA_MATRIX_KEYS = ("camera_matrix", "K", "mtx")
DISTORTION_KEYS = ("dist_coeffs", "dist", "D")
# The existing project uses homography_matrix; the other names cover common formats.
HOMOGRAPHY_KEYS = ("H", "homography", "M", "matrix", "homography_matrix")


def load_npz_value(path: Path, candidate_keys: Sequence[str], label: str) -> np.ndarray:
    """Load the first matching array and report all available keys on failure."""
    if not path.exists():
        raise FileNotFoundError(f"{label} NPZ file not found: {path}")

    with np.load(path) as data:
        for key in candidate_keys:
            if key in data:
                print(f"Loaded {label}: {path} (key: {key})")
                return np.asarray(data[key]).copy()
        available_keys = list(data.files)

    raise KeyError(
        f"No supported key for {label} in {path}. "
        f"Expected one of {list(candidate_keys)}. Available keys: {available_keys}"
    )


def load_calibration() -> tuple[np.ndarray, np.ndarray]:
    """Load and validate the intrinsic camera matrix and distortion coefficients."""
    camera_matrix = load_npz_value(
        CALIBRATION_PATH, CAMERA_MATRIX_KEYS, "camera matrix"
    ).astype(np.float64)
    dist_coeffs = load_npz_value(
        CALIBRATION_PATH, DISTORTION_KEYS, "distortion coefficients"
    ).astype(np.float64)

    if camera_matrix.shape != (3, 3):
        raise ValueError(
            f"Camera matrix must have shape (3, 3), got {camera_matrix.shape}"
        )
    if dist_coeffs.size < 4:
        raise ValueError(
            f"Distortion coefficients must contain at least 4 values, got {dist_coeffs.shape}"
        )
    return camera_matrix, dist_coeffs


def load_homography() -> np.ndarray:
    """Load and validate the image-to-BEV homography matrix."""
    homography = load_npz_value(
        HOMOGRAPHY_PATH, HOMOGRAPHY_KEYS, "BEV homography"
    ).astype(np.float64)
    if homography.shape != (3, 3):
        raise ValueError(f"Homography must have shape (3, 3), got {homography.shape}")
    return homography


def configure_camera(camera: cv2.VideoCapture) -> None:
    """Request MJPG encoding and the calibrated 1920x1080 capture resolution."""
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)


def main() -> int:
    """Show live camera/BEV views and save the BEV frame when s is pressed."""
    try:
        camera_matrix, dist_coeffs = load_calibration()
        homography = load_homography()
    except (FileNotFoundError, KeyError, ValueError, OSError) as error:
        print(f"ERROR: {error}")
        return 1

    camera = cv2.VideoCapture(CAMERA_ID)
    if not camera.isOpened():
        camera.release()
        print(f"ERROR: Could not open USB camera with camera_id={CAMERA_ID}")
        return 1

    configure_camera(camera)
    actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Opened camera_id={CAMERA_ID}")
    print(f"Requested camera size: {CAMERA_WIDTH} x {CAMERA_HEIGHT}")
    print(f"Actual camera size: {actual_width} x {actual_height}")
    if (actual_width, actual_height) != (CAMERA_WIDTH, CAMERA_HEIGHT):
        print("WARNING: Camera did not accept the requested 1920 x 1080 resolution.")
    print("Press 's' to save camera_bev.png; press 'q' or ESC to quit.")

    try:
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                print("ERROR: Failed to read a frame from the USB camera")
                return 1

            # Processing order must match calibration: raw -> undistort -> BEV warp.
            undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
            bev = cv2.warpPerspective(
                undistorted,
                homography,
                (BEV_WIDTH, BEV_HEIGHT),
                flags=cv2.INTER_LINEAR,
            )

            # A reduced source preview fits alongside the 1600x800 BEV window.
            preview = cv2.resize(frame, (960, 540), interpolation=cv2.INTER_AREA)
            cv2.imshow("USB Camera (preview)", preview)
            cv2.imshow("Camera BEV 1600x800", bev)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                if not cv2.imwrite(str(OUTPUT_PATH), bev):
                    print(f"ERROR: Failed to save BEV image: {OUTPUT_PATH}")
                    continue
                print(f"Saved BEV image: {OUTPUT_PATH}")
                print(f"Image size: {BEV_WIDTH} x {BEV_HEIGHT}")
    finally:
        camera.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
