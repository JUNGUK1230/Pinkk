"""Record lens-corrected Camera BEV video for YOLO dataset collection.

Run:
    cd ~/PINKK/src/central_control/camera_tools/first_map
    /usr/bin/python3 capture_bev_image.py

Controls:
    SPACE: start or stop BEV-only video recording
    q/ESC: stop recording and quit

The raw and undistorted camera frames are used only in memory. They are never
written to disk. All paths are resolved inside the PINKK repository.
"""

from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Camera settings: change CAMERA_ID here when the USB device number changes.
# These values must match the resolution used during camera calibration.
# ---------------------------------------------------------------------------
CAMERA_ID = 2
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FPS = 30
WARMUP_FRAMES = 10
DISPLAY_WIDTH = 1200

# MP4V is available in the standard OpenCV/FFmpeg installation and produces a
# video that CVAT and the frame extraction script can read without extra tools.
VIDEO_CODEC = "mp4v"
VIDEO_EXTENSION = ".mp4"

# Every input and output stays beside this script in the PINKK repository.
SCRIPT_DIR = Path(__file__).resolve().parent
CALIBRATION_PATH = SCRIPT_DIR / "camera_calibration.npz"
HOMOGRAPHY_PATH = SCRIPT_DIR / "bev_homography.npz"
RECORDING_DIR = SCRIPT_DIR / "bev_recordings"


def _require_key(data: np.lib.npyio.NpzFile, key: str, path: Path) -> np.ndarray:
    """Return a required NPZ value and show available keys when it is absent."""
    if key not in data:
        raise KeyError(
            f"Required key '{key}' is missing from {path}. "
            f"Available keys: {list(data.files)}"
        )
    return np.asarray(data[key]).copy()


def load_camera_calibration(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load and validate the camera intrinsic and distortion parameters."""
    if not path.exists():
        raise FileNotFoundError(f"Camera calibration file not found: {path}")

    with np.load(path) as data:
        camera_matrix = _require_key(data, "camera_matrix", path).astype(np.float64)
        dist_coeffs = _require_key(data, "dist_coeffs", path).astype(np.float64)

    if camera_matrix.shape != (3, 3):
        raise ValueError(
            f"camera_matrix must have shape (3, 3), got {camera_matrix.shape}"
        )
    if dist_coeffs.size < 4:
        raise ValueError(
            f"dist_coeffs must contain at least 4 values, got {dist_coeffs.shape}"
        )
    return camera_matrix, dist_coeffs


def load_bev_homography(path: Path) -> tuple[np.ndarray, int, int]:
    """Load the image-to-BEV homography and calibrated output size."""
    if not path.exists():
        raise FileNotFoundError(f"BEV homography file not found: {path}")

    with np.load(path) as data:
        homography = _require_key(data, "homography_matrix", path).astype(np.float64)
        bev_width = int(_require_key(data, "bev_width", path).item())
        bev_height = int(_require_key(data, "bev_height", path).item())

    if homography.shape != (3, 3):
        raise ValueError(
            f"homography_matrix must have shape (3, 3), got {homography.shape}"
        )
    if bev_width <= 0 or bev_height <= 0:
        raise ValueError(f"Invalid BEV size: {bev_width} x {bev_height}")
    return homography, bev_width, bev_height


def open_camera() -> cv2.VideoCapture:
    """Open the V4L2 camera and request MJPG 1920x1080 at 30 FPS."""
    camera = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(
            f"Could not open USB camera_id={CAMERA_ID}. "
            "Check the device with: ls -l /dev/video*"
        )

    # Set MJPG first because many USB cameras require it for 1080p/30 FPS.
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    camera.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    return camera


def start_bev_recording(
    width: int,
    height: int,
    fps: float,
) -> tuple[cv2.VideoWriter, Path]:
    """Create a timestamped BEV-only MP4 writer."""
    RECORDING_DIR.mkdir(parents=True, exist_ok=True)
    # Microseconds prevent overwriting when recording is restarted quickly.
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    video_path = RECORDING_DIR / f"bev_{timestamp}{VIDEO_EXTENSION}"
    writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*VIDEO_CODEC),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        writer.release()
        raise RuntimeError(
            f"Could not create BEV video: {video_path} "
            f"(codec={VIDEO_CODEC}, size={width}x{height}, fps={fps:.1f})"
        )
    return writer, video_path


def main() -> int:
    """Show the live BEV and record only BEV frames while recording is active."""
    try:
        camera_matrix, dist_coeffs = load_camera_calibration(CALIBRATION_PATH)
        homography, bev_width, bev_height = load_bev_homography(HOMOGRAPHY_PATH)
        camera = open_camera()
    except (FileNotFoundError, KeyError, ValueError, RuntimeError, OSError) as error:
        print(f"ERROR: {error}")
        return 1

    actual_width = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = camera.get(cv2.CAP_PROP_FPS)
    recording_fps = actual_fps if actual_fps > 0 else float(CAMERA_FPS)

    print("=" * 60)
    print("PINKK BEV-only Video Recorder")
    print("=" * 60)
    print(f"Camera ID       : {CAMERA_ID}")
    print(f"Requested input : {CAMERA_WIDTH} x {CAMERA_HEIGHT} @ {CAMERA_FPS} FPS")
    print(f"Actual input    : {actual_width} x {actual_height} @ {actual_fps:.1f} FPS")
    print(f"BEV video       : {bev_width} x {bev_height} @ {recording_fps:.1f} FPS")
    print(f"Recording folder: {RECORDING_DIR}")

    if (actual_width, actual_height) != (CAMERA_WIDTH, CAMERA_HEIGHT):
        camera.release()
        print(
            "ERROR: Camera did not accept the calibrated 1920 x 1080 resolution. "
            f"Actual resolution is {actual_width} x {actual_height}."
        )
        return 1

    writer: cv2.VideoWriter | None = None
    video_path: Path | None = None
    recorded_frames = 0

    try:
        # Discard initial frames while auto exposure and white balance settle.
        for _ in range(WARMUP_FRAMES):
            ok, _ = camera.read()
            if not ok:
                print("ERROR: Failed to read a camera warm-up frame")
                return 1

        print("SPACE: start/stop recording | q or ESC: quit")

        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                print("ERROR: Failed to read a camera frame")
                return 1

            # Raw and undistorted frames remain in memory and are not saved.
            undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs)
            bev = cv2.warpPerspective(
                undistorted,
                homography,
                (bev_width, bev_height),
                flags=cv2.INTER_LINEAR,
            )

            # Write the clean BEV before adding UI text to the preview.
            if writer is not None:
                writer.write(bev)
                recorded_frames += 1

            display_height = max(1, round(bev_height * DISPLAY_WIDTH / bev_width))
            bev_display = cv2.resize(
                bev,
                (DISPLAY_WIDTH, display_height),
                interpolation=cv2.INTER_AREA,
            )
            if writer is not None:
                cv2.circle(bev_display, (25, 30), 10, (0, 0, 255), -1)
                status = f"REC {recorded_frames} frames | SPACE: Stop | Q: Quit"
                status_color = (0, 0, 255)
            else:
                status = "SPACE: Start recording | Q/ESC: Quit"
                status_color = (0, 255, 255)
            cv2.putText(
                bev_display,
                status,
                (45, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                status_color,
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("PINKK BEV Video Recorder", bev_display)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == 32:  # SPACE toggles recording.
                if writer is None:
                    try:
                        writer, video_path = start_bev_recording(
                            bev_width,
                            bev_height,
                            recording_fps,
                        )
                    except RuntimeError as error:
                        print(f"ERROR: {error}")
                        continue
                    recorded_frames = 0
                    print(f"Recording started: {video_path}")
                else:
                    writer.release()
                    writer = None
                    print(f"Recording stopped: {video_path}")
                    print(f"Recorded BEV frames: {recorded_frames}")
    finally:
        if writer is not None:
            writer.release()
            print(f"Recording saved: {video_path}")
            print(f"Recorded BEV frames: {recorded_frames}")
        camera.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
