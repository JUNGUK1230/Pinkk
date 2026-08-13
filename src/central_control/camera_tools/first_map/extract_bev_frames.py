"""Uniformly extract CVAT labeling images from a recorded BEV video.

By default, this script selects the newest video in ``bev_recordings`` and
extracts up to 1500 frames spread across the whole recording.

Run:
    cd ~/PINKK/src/central_control/camera_tools/first_map
    /usr/bin/python3 extract_bev_frames.py --count 1500
"""

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
RECORDING_DIR = SCRIPT_DIR / "bev_recordings"
DATASET_DIR = SCRIPT_DIR / "bev_dataset"
SUPPORTED_VIDEO_EXTENSIONS = (".mp4", ".avi", ".mov", ".mkv")


def parse_args() -> argparse.Namespace:
    """Parse command-line options while keeping useful project defaults."""
    parser = argparse.ArgumentParser(
        description="Extract evenly spaced BEV frames for CVAT labeling."
    )
    parser.add_argument(
        "--video",
        type=Path,
        help="Input video. If omitted, the newest video in bev_recordings is used.",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=100,
        help="Maximum number of frames to extract (default: 1500).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Output folder. Default: bev_dataset/cvat_images/<video_name>.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=95,
        help="JPEG quality from 1 to 100 (default: 95).",
    )
    return parser.parse_args()


def find_latest_video() -> Path:
    """Return the most recently modified supported BEV recording."""
    if not RECORDING_DIR.exists():
        raise FileNotFoundError(f"Recording folder not found: {RECORDING_DIR}")
    videos = [
        path
        for path in RECORDING_DIR.iterdir()
        if path.is_file() and path.suffix.lower() in SUPPORTED_VIDEO_EXTENSIONS
    ]
    if not videos:
        raise FileNotFoundError(f"No BEV video found in: {RECORDING_DIR}")
    return max(videos, key=lambda path: path.stat().st_mtime)


def get_total_frames(capture: cv2.VideoCapture) -> int:
    """Read the video frame count, falling back to a full scan if unavailable."""
    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    if total_frames > 0:
        return total_frames

    total_frames = 0
    while True:
        ok, _ = capture.read()
        if not ok:
            break
        total_frames += 1
    capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
    return total_frames


def select_frame_indices(total_frames: int, requested_count: int) -> list[int]:
    """Select unique frame indices uniformly over the entire video."""
    if requested_count <= 0:
        raise ValueError(f"--count must be greater than zero, got {requested_count}")
    if total_frames <= 0:
        raise ValueError("The input video contains no readable frames")

    output_count = min(total_frames, requested_count)
    indices = np.linspace(0, total_frames - 1, output_count, dtype=np.int64)
    return np.unique(indices).astype(int).tolist()


def extract_frames(
    video_path: Path,
    output_dir: Path,
    requested_count: int,
    jpeg_quality: int,
) -> tuple[int, int]:
    """Extract selected frames and write a source-frame manifest for CVAT."""
    if not video_path.exists():
        raise FileNotFoundError(f"Input video not found: {video_path}")
    if not 1 <= jpeg_quality <= 100:
        raise ValueError(
            f"--jpeg-quality must be between 1 and 100, got {jpeg_quality}"
        )

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f"Could not open video: {video_path}")

    try:
        total_frames = get_total_frames(capture)
        selected_indices = select_frame_indices(total_frames, requested_count)
        fps = capture.get(cv2.CAP_PROP_FPS)
        output_dir.mkdir(parents=True, exist_ok=True)
        manifest_rows: list[tuple[str, int, float]] = []

        for output_index, source_index in enumerate(selected_indices):
            capture.set(cv2.CAP_PROP_POS_FRAMES, source_index)
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError(f"Could not read source frame {source_index}")

            filename = f"{video_path.stem}_{output_index:04d}.jpg"
            image_path = output_dir / filename
            saved = cv2.imwrite(
                str(image_path),
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
            )
            if not saved:
                raise OSError(f"Failed to save extracted frame: {image_path}")
            time_seconds = source_index / fps if fps > 0 else 0.0
            manifest_rows.append((filename, source_index, time_seconds))

        manifest_path = output_dir / "frames_manifest.csv"
        with manifest_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(("filename", "source_frame", "time_seconds"))
            for filename, source_index, time_seconds in manifest_rows:
                writer.writerow((filename, source_index, f"{time_seconds:.6f}"))
    finally:
        capture.release()

    return total_frames, len(selected_indices)


def main() -> int:
    """Resolve defaults, extract the requested dataset images, and report paths."""
    args = parse_args()
    try:
        video_path = args.video.resolve() if args.video else find_latest_video()
        output_dir = (
            args.output_dir.resolve()
            if args.output_dir
            else DATASET_DIR / "cvat_images" / video_path.stem
        )
        total_frames, extracted_count = extract_frames(
            video_path,
            output_dir,
            args.count,
            args.jpeg_quality,
        )
    except (FileNotFoundError, ValueError, RuntimeError, OSError) as error:
        print(f"ERROR: {error}")
        return 1

    print(f"Input video: {video_path}")
    print(f"Video frames: {total_frames}")
    print(f"Extracted frames: {extracted_count}")
    if extracted_count < args.count:
        print(
            f"WARNING: Requested {args.count} images, but the video only contains "
            f"{total_frames} unique frames."
        )
    print(f"CVAT image folder: {output_dir}")
    print(f"Manifest: {output_dir / 'frames_manifest.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
