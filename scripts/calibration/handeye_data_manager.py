#!/usr/bin/env python3
"""Permanent, Git-trackable storage for Easy Handeye2 calibration runs."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import re
import shutil
import socket
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import yaml


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(
    os.environ.get("PINKK_REPO_ROOT", str(SCRIPT_DIR.parents[1]))
).expanduser().resolve()
DATA_ROOT = Path(
    os.environ.get(
        "PINKK_HANDEYE_DATA_ROOT",
        str(
            REPO_ROOT
            / "src/robot_arm/robot_camera/handeye_calibration_1828/data"
        ),
    )
).expanduser().resolve()
RUNS_ROOT = DATA_ROOT / "runs"
COMPARISONS_ROOT = DATA_ROOT / "comparisons"
ACTIVE_ROOT = DATA_ROOT / "active"
USB_HANDEYE_CONFIG = Path(
    os.environ.get(
        "PINKK_USB_HANDEYE_CONFIG",
        str(REPO_ROOT / "ros2_ws/src/pinkk_usb_insertion/config/handeye.yaml"),
    )
).expanduser().resolve()
INSTALLED_USB_HANDEYE_CONFIG = Path(
    os.environ.get(
        "PINKK_INSTALLED_USB_HANDEYE_CONFIG",
        str(
            Path.home()
            / "mycobot_moveit_ws/install_pinkk/pinkk_usb_insertion"
            / "share/pinkk_usb_insertion/config/handeye.yaml"
        ),
    )
).expanduser().resolve()
LEGACY_MATRIX = DATA_ROOT / "T_flange_camera.npy"
LEGACY_EASY_MATRIX = DATA_ROOT / "T_flange_camera_easy_handeye.npy"


def now() -> datetime:
    return datetime.now().astimezone()


def iso_now() -> str:
    return now().isoformat()


def slugify(value: str) -> str:
    slug = re.sub(r"[^A-Za-z0-9_-]+", "_", value.strip()).strip("_").lower()
    return slug or "run"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"JSON 최상위 항목은 object여야 합니다: {path}")
    return data


def write_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def git_value(*arguments: str) -> str:
    try:
        result = subprocess.run(
            ("git", *arguments),
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        return result.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def unique_directory(root: Path, stem: str) -> Path:
    candidate = root / stem
    suffix = 2
    while candidate.exists():
        candidate = root / f"{stem}_{suffix:02d}"
        suffix += 1
    candidate.mkdir(parents=True)
    return candidate


def resolve_run(selector: str) -> Path:
    candidate = Path(selector).expanduser()
    if candidate.is_file() and candidate.suffix == ".calib":
        return candidate.resolve().parent
    if candidate.is_dir() and (candidate / "calibration.calib").is_file():
        return candidate.resolve()

    direct = RUNS_ROOT / selector
    if direct.is_dir():
        return direct.resolve()

    matches = sorted(
        path for path in RUNS_ROOT.glob(f"*{selector}*") if path.is_dir()
    )
    if not matches:
        raise FileNotFoundError(f"run을 찾을 수 없습니다: {selector}")
    if len(matches) > 1:
        names = ", ".join(path.name for path in matches)
        raise ValueError(f"run 이름이 모호합니다: {selector} -> {names}")
    return matches[0].resolve()


def resolve_calibration(selector: str) -> Path:
    candidate = Path(selector).expanduser()
    if candidate.is_file():
        return candidate.resolve()
    if selector == "active":
        active = ACTIVE_ROOT / "calibration.calib"
        if not active.is_file():
            raise FileNotFoundError("활성 calibration이 없습니다")
        return active.resolve()
    run = resolve_run(selector)
    calibration = run / "calibration.calib"
    if not calibration.is_file():
        raise FileNotFoundError(f"run에 calibration이 없습니다: {run.name}")
    return calibration.resolve()


def load_calibration(path: Path) -> tuple[dict[str, Any], np.ndarray]:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"올바른 calibration YAML이 아닙니다: {path}")
    parameters = data.get("parameters", {})
    expected = {
        "calibration_type": "eye_in_hand",
        "robot_base_frame": "g_base",
        "robot_effector_frame": "joint6_flange",
        "tracking_base_frame": "camera_optical_frame",
        "tracking_marker_frame": "charuco_board",
    }
    mismatches = [
        f"{key}={parameters.get(key)!r}"
        for key, value in expected.items()
        if parameters.get(key) != value
    ]
    if mismatches:
        raise ValueError(
            f"calibration frame/type 불일치: {path}: {', '.join(mismatches)}"
        )
    try:
        translation = data["transform"]["translation"]
        rotation = data["transform"]["rotation"]
        xyz = np.array([float(translation[axis]) for axis in "xyz"])
        quaternion = np.array([float(rotation[axis]) for axis in "xyzw"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"calibration transform 형식 오류: {path}") from error
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12 or not np.isfinite(xyz).all():
        raise ValueError(f"calibration transform 값 오류: {path}")
    x, y, z, w = quaternion / norm
    matrix = np.eye(4, dtype=float)
    matrix[:3, :3] = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    matrix[:3, 3] = xyz
    if not np.isclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=1e-6):
        raise ValueError(f"calibration rotation 행렬 오류: {path}")
    return data, matrix


def calibration_summary(path: Path) -> dict[str, Any]:
    data, matrix = load_calibration(path)
    transform = data["transform"]
    return {
        "parameters": data.get("parameters", {}),
        "transform": transform,
        "matrix_4x4": matrix.tolist(),
        "sha256": sha256(path),
    }


def sample_count(path: Path) -> int:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    samples = data.get("samples", []) if isinstance(data, dict) else []
    if not isinstance(samples, list):
        raise ValueError(f"samples 형식 오류: {path}")
    return len(samples)


def quaternion_to_matrix(rotation: dict[str, Any]) -> np.ndarray:
    try:
        quaternion = np.array([float(rotation[axis]) for axis in "xyzw"])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("sample quaternion 형식 오류") from error
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12 or not np.isfinite(quaternion).all():
        raise ValueError("sample quaternion 값 오류")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Return a normalized quaternion in ROS xyzw order."""
    candidates = np.array(
        [
            1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2],
            1.0 - matrix[0, 0] + matrix[1, 1] - matrix[2, 2],
            1.0 - matrix[0, 0] - matrix[1, 1] + matrix[2, 2],
            1.0 + np.trace(matrix),
        ]
    )
    index = int(np.argmax(candidates))
    value = max(float(candidates[index]), 0.0)
    component = 0.5 * np.sqrt(value)
    if component <= 1e-12:
        raise ValueError("계산된 rotation을 quaternion으로 변환할 수 없습니다")
    denominator = 4.0 * component
    if index == 0:
        quaternion = np.array(
            [
                component,
                (matrix[0, 1] + matrix[1, 0]) / denominator,
                (matrix[0, 2] + matrix[2, 0]) / denominator,
                (matrix[2, 1] - matrix[1, 2]) / denominator,
            ]
        )
    elif index == 1:
        quaternion = np.array(
            [
                (matrix[0, 1] + matrix[1, 0]) / denominator,
                component,
                (matrix[1, 2] + matrix[2, 1]) / denominator,
                (matrix[0, 2] - matrix[2, 0]) / denominator,
            ]
        )
    elif index == 2:
        quaternion = np.array(
            [
                (matrix[0, 2] + matrix[2, 0]) / denominator,
                (matrix[1, 2] + matrix[2, 1]) / denominator,
                component,
                (matrix[1, 0] - matrix[0, 1]) / denominator,
            ]
        )
    else:
        quaternion = np.array(
            [
                (matrix[2, 1] - matrix[1, 2]) / denominator,
                (matrix[0, 2] - matrix[2, 0]) / denominator,
                (matrix[1, 0] - matrix[0, 1]) / denominator,
                component,
            ]
        )
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0:
        quaternion *= -1
    return quaternion


def compute_run(arguments: argparse.Namespace) -> int:
    """Recover a calibration directly from a run's preserved samples."""
    try:
        import cv2
    except ImportError as error:
        raise RuntimeError("OpenCV를 불러올 수 없습니다") from error
    if not hasattr(cv2, "calibrateHandEye"):
        raise RuntimeError(
            f"현재 OpenCV {getattr(cv2, '__version__', 'unknown')}에는 "
            "calibrateHandEye가 없습니다"
        )

    run = resolve_run(arguments.run)
    samples_path = run / "samples.samples"
    calibration_path = run / "calibration.calib"
    if not samples_path.is_file():
        raise FileNotFoundError(f"run에 samples가 없습니다: {run.name}")
    if calibration_path.exists():
        raise ValueError(
            f"기존 calibration을 덮어쓰지 않습니다: {calibration_path}"
        )

    document = yaml.safe_load(samples_path.read_text(encoding="utf-8"))
    samples = document.get("samples", []) if isinstance(document, dict) else []
    if not isinstance(samples, list) or len(samples) < 3:
        raise ValueError(f"계산할 sample이 3개 이상 필요합니다: {samples_path}")

    robot_rotations: list[np.ndarray] = []
    robot_translations: list[np.ndarray] = []
    tracking_rotations: list[np.ndarray] = []
    tracking_translations: list[np.ndarray] = []
    for index, sample in enumerate(samples, start=1):
        try:
            robot = sample["robot"]
            tracking = sample["tracking"]
            robot_rotations.append(quaternion_to_matrix(robot["rotation"]))
            tracking_rotations.append(quaternion_to_matrix(tracking["rotation"]))
            robot_translations.append(
                np.array([float(robot["translation"][axis]) for axis in "xyz"])
            )
            tracking_translations.append(
                np.array([float(tracking["translation"][axis]) for axis in "xyz"])
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"sample {index} transform 형식 오류") from error

    rotation, translation = cv2.calibrateHandEye(
        robot_rotations,
        robot_translations,
        tracking_rotations,
        tracking_translations,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    translation = np.asarray(translation, dtype=float).reshape(3)
    quaternion = matrix_to_quaternion(rotation)

    metadata_path = run / "metadata.json"
    metadata = read_json(metadata_path)
    frames = metadata.get("collection", {}).get("frames", {})
    easy_name = metadata.get("collection", {}).get(
        "easy_handeye_name", "pinkk_eye_in_hand"
    )
    calibration = {
        "parameters": {
            "name": easy_name,
            "calibration_type": "eye_in_hand",
            "robot_base_frame": frames.get("robot_base", "g_base"),
            "robot_effector_frame": frames.get(
                "robot_effector", "joint6_flange"
            ),
            "tracking_base_frame": frames.get(
                "tracking_base", "camera_optical_frame"
            ),
            "tracking_marker_frame": frames.get(
                "tracking_marker", "charuco_board"
            ),
            "freehand_robot_movement": True,
            "move_group_namespace": "/",
            "move_group": "manipulator",
        },
        "transform": {
            "translation": dict(zip("xyz", map(float, translation))),
            "rotation": dict(zip("xyzw", map(float, quaternion))),
        },
    }
    calibration_path.write_text(
        yaml.safe_dump(calibration, sort_keys=False),
        encoding="utf-8",
    )
    metadata.setdefault("collection", {})["algorithm"] = "OpenCV/Tsai-Lenz"
    metadata["recovered_from_samples_at"] = iso_now()
    metadata["recovery_opencv"] = {
        "version": str(cv2.__version__),
        "module": str(cv2.__file__),
    }
    write_json(metadata_path, metadata)
    print(
        f"{len(samples)}개 sample로 계산 완료: {calibration_path}\n"
        f"OpenCV={cv2.__version__} ({cv2.__file__})"
    )
    return archive_run(argparse.Namespace(run=str(run), easy_name=""))


def create_run(arguments: argparse.Namespace) -> int:
    created = now()
    run_id = f"{created.strftime('%Y%m%d_%H%M%S')}_{slugify(arguments.label)}"
    git_metadata = {
        "branch": git_value("branch", "--show-current"),
        "commit": git_value("rev-parse", "HEAD"),
        "dirty": bool(git_value("status", "--porcelain")),
    }
    run = unique_directory(RUNS_ROOT, run_id)
    metadata = {
        "schema_version": 1,
        "run_id": run.name,
        "label": arguments.label,
        "created_at": created.isoformat(),
        "created_epoch": created.timestamp(),
        "status": "collecting",
        "git": git_metadata,
        "system": {
            "hostname": socket.gethostname(),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", ""),
        },
        "collection": {
            "easy_handeye_name": arguments.easy_name,
            "algorithm": "OpenCV/Tsai-Lenz",
            "target_samples": arguments.target_samples,
            "minimum_samples": arguments.minimum_samples,
            "frames": {
                "robot_base": "g_base",
                "robot_effector": "joint6_flange",
                "tracking_base": "camera_optical_frame",
                "tracking_marker": "charuco_board",
            },
        },
        "artifacts": {},
    }
    intrinsics = (
        REPO_ROOT
        / "src/robot_arm/robot_camera/camera_calibration/results/intrinsics.npz"
    )
    if intrinsics.is_file():
        metadata["intrinsics"] = {
            "path": str(intrinsics.relative_to(REPO_ROOT)),
            "sha256": sha256(intrinsics),
        }
    write_json(run / "metadata.json", metadata)
    print(run)
    return 0


def fresh_source(path: Path, created_epoch: float) -> bool:
    return path.is_file() and path.stat().st_mtime >= created_epoch - 2.0


def archive_run(arguments: argparse.Namespace) -> int:
    run = resolve_run(arguments.run)
    metadata_path = run / "metadata.json"
    metadata = read_json(metadata_path)
    easy_name = arguments.easy_name or metadata["collection"]["easy_handeye_name"]
    created_epoch = float(metadata.get("created_epoch", 0.0))
    ros_root = Path(
        os.environ.get(
            "PINKK_EASY_HANDEYE_ROOT",
            str(Path.home() / ".ros2/easy_handeye2"),
        )
    ).expanduser().resolve()
    sources = {
        "samples": ros_root / "samples" / f"{easy_name}.samples",
        "calibration": ros_root / "calibrations" / f"{easy_name}.calib",
    }
    destinations = {
        "samples": run / "samples.samples",
        "calibration": run / "calibration.calib",
    }

    copied: list[str] = []
    skipped_stale: list[str] = []
    for kind, source in sources.items():
        destination = destinations[kind]
        if destination.is_file():
            continue
        if not fresh_source(source, created_epoch):
            skipped_stale.append(kind)
            continue
        shutil.copy2(source, destination)
        copied.append(kind)

    artifacts = metadata.setdefault("artifacts", {})
    samples = destinations["samples"]
    calibration = destinations["calibration"]
    if samples.is_file():
        artifacts["samples"] = {
            "file": samples.name,
            "count": sample_count(samples),
            "sha256": sha256(samples),
        }
    if calibration.is_file():
        _, matrix = load_calibration(calibration)
        matrix_path = run / "T_flange_camera.npy"
        np.save(matrix_path, matrix)
        artifacts["calibration"] = {
            "file": calibration.name,
            **calibration_summary(calibration),
        }
        artifacts["matrix"] = {
            "file": matrix_path.name,
            "sha256": sha256(matrix_path),
        }

    if samples.is_file() and calibration.is_file():
        metadata["status"] = "complete"
    elif samples.is_file():
        metadata["status"] = "samples_only"
    elif calibration.is_file():
        metadata["status"] = "calibration_only"
    else:
        metadata["status"] = "no_new_artifacts"
    metadata["archived_at"] = iso_now()
    metadata["archive_source"] = str(ros_root)
    write_json(metadata_path, metadata)

    print(
        f"run={run.name}, status={metadata['status']}, "
        f"copied={copied or 'none'}, stale_or_missing={skipped_stale or 'none'}"
    )
    return 0 if artifacts else 3


def list_runs(_: argparse.Namespace) -> int:
    rows: list[tuple[str, str, str, str]] = []
    for run in sorted(RUNS_ROOT.iterdir()) if RUNS_ROOT.is_dir() else []:
        if not run.is_dir():
            continue
        metadata_path = run / "metadata.json"
        metadata = read_json(metadata_path) if metadata_path.is_file() else {}
        samples = metadata.get("artifacts", {}).get("samples", {}).get("count", "-")
        rows.append(
            (
                run.name,
                str(metadata.get("status", "unmanaged")),
                str(samples),
                str(metadata.get("label", "")),
            )
        )
    print(f"{'RUN':48} {'STATUS':18} {'SAMPLES':8} LABEL")
    for run_id, status, samples, label in rows:
        print(f"{run_id:48} {status:18} {samples:8} {label}")
    return 0


def create_comparison(arguments: argparse.Namespace) -> int:
    old = resolve_calibration(arguments.old)
    new = resolve_calibration(arguments.new)
    stamp = now().strftime("%Y%m%d_%H%M%S")
    name = f"{stamp}_{slugify(old.parent.name)}_vs_{slugify(new.parent.name)}"
    comparison = unique_directory(COMPARISONS_ROOT, name)
    metadata = {
        "schema_version": 1,
        "comparison_id": comparison.name,
        "created_at": iso_now(),
        "status": "running",
        "old_calibration": str(old.relative_to(REPO_ROOT)),
        "new_calibration": str(new.relative_to(REPO_ROOT)),
        "pose_limit": arguments.pose_limit,
        "git_commit": git_value("rev-parse", "HEAD"),
    }
    write_json(comparison / "metadata.json", metadata)
    print(comparison / "measurements.csv")
    return 0


def finalize_comparison(arguments: argparse.Namespace) -> int:
    csv_path = Path(arguments.csv).expanduser().resolve()
    comparison = csv_path.parent
    metadata_path = comparison / "metadata.json"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"comparison metadata가 없습니다: {metadata_path}")
    summary_path = csv_path.with_suffix(".summary.json")
    metadata = read_json(metadata_path)
    if not csv_path.is_file() or not summary_path.is_file():
        metadata["status"] = "failed_or_incomplete"
        metadata["finalized_at"] = iso_now()
        write_json(metadata_path, metadata)
        print(f"비교 결과가 완전하지 않습니다: {comparison}")
        return 4
    with csv_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    metadata.update(
        {
            "status": "complete",
            "finalized_at": iso_now(),
            "pose_count": len({row["pose_index"] for row in rows}),
            "artifacts": {
                "measurements": {
                    "file": csv_path.name,
                    "sha256": sha256(csv_path),
                },
                "summary": {
                    "file": summary_path.name,
                    "sha256": sha256(summary_path),
                },
            },
        }
    )
    write_json(metadata_path, metadata)
    print(f"comparison 저장 완료: {comparison}")
    return 0


def write_active_files(run: Path, calibration: Path, matrix: np.ndarray) -> None:
    ACTIVE_ROOT.mkdir(parents=True, exist_ok=True)
    shutil.copy2(calibration, ACTIVE_ROOT / "calibration.calib")
    np.save(ACTIVE_ROOT / "T_flange_camera.npy", matrix)
    np.save(LEGACY_MATRIX, matrix)
    np.save(LEGACY_EASY_MATRIX, matrix)
    summary = calibration_summary(calibration)
    manifest = {
        "schema_version": 1,
        "activated_at": iso_now(),
        "run_id": run.name,
        "source_calibration": str(calibration.relative_to(REPO_ROOT)),
        "calibration_sha256": summary["sha256"],
        "matrix_sha256": sha256(ACTIVE_ROOT / "T_flange_camera.npy"),
    }
    write_json(ACTIVE_ROOT / "manifest.json", manifest)

    transform = summary["transform"]
    usb_data = {
        "handeye": {
            "source_file": str(
                (ACTIVE_ROOT / "T_flange_camera.npy").relative_to(REPO_ROOT)
            ),
            "source_run": run.name,
            "parent_frame": "joint6_flange",
            "child_frame": "camera_optical_frame",
            "matrix_4x4": matrix.tolist(),
            "translation_m": transform["translation"],
            "quaternion_xyzw": transform["rotation"],
            "calibrated": True,
            "source": "easy_handeye2",
        }
    }
    USB_HANDEYE_CONFIG.write_text(
        yaml.safe_dump(usb_data, sort_keys=False),
        encoding="utf-8",
    )
    if INSTALLED_USB_HANDEYE_CONFIG.parent.is_dir():
        shutil.copy2(USB_HANDEYE_CONFIG, INSTALLED_USB_HANDEYE_CONFIG)


def activate_run(arguments: argparse.Namespace) -> int:
    run = resolve_run(arguments.run)
    calibration = run / "calibration.calib"
    if not calibration.is_file():
        raise FileNotFoundError(f"run에 calibration이 없습니다: {run.name}")
    _, matrix = load_calibration(calibration)
    write_active_files(run, calibration, matrix)
    print(f"활성 Hand-eye 변경 완료: {run.name}")
    print(f"USB 설정 동기화: {USB_HANDEYE_CONFIG}")
    if INSTALLED_USB_HANDEYE_CONFIG.is_file():
        print(f"설치 overlay 동기화: {INSTALLED_USB_HANDEYE_CONFIG}")
    return 0


def show_active(_: argparse.Namespace) -> int:
    manifest = ACTIVE_ROOT / "manifest.json"
    if not manifest.is_file():
        print("활성 Hand-eye manifest가 없습니다")
        return 2
    print(manifest.read_text(encoding="utf-8"), end="")
    return 0


def active_values(arguments: argparse.Namespace) -> int:
    calibration = resolve_calibration(arguments.selector)
    data, _ = load_calibration(calibration)
    translation = data["transform"]["translation"]
    rotation = data["transform"]["rotation"]
    values = [translation[axis] for axis in "xyz"] + [rotation[axis] for axis in "xyzw"]
    print(" ".join(str(float(value)) for value in values))
    return 0


def resolve_command(arguments: argparse.Namespace) -> int:
    print(resolve_calibration(arguments.selector))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    create = subparsers.add_parser("create", help="새 수집 run 폴더 생성")
    create.add_argument("--label", required=True)
    create.add_argument("--target-samples", type=int, required=True)
    create.add_argument("--minimum-samples", type=int, required=True)
    create.add_argument("--easy-name", default="pinkk_eye_in_hand")
    create.set_defaults(handler=create_run)

    archive = subparsers.add_parser("archive", help="Easy Handeye2 결과를 run에 보관")
    archive.add_argument("run")
    archive.add_argument("--easy-name", default="")
    archive.set_defaults(handler=archive_run)

    compute = subparsers.add_parser(
        "compute-run", help="보존된 samples로 calibration 복구 계산"
    )
    compute.add_argument("run")
    compute.set_defaults(handler=compute_run)

    listing = subparsers.add_parser("list", help="저장된 run 목록")
    listing.set_defaults(handler=list_runs)

    comparison = subparsers.add_parser(
        "create-comparison", help="Git 추적 비교 폴더 생성"
    )
    comparison.add_argument("--old", required=True)
    comparison.add_argument("--new", required=True)
    comparison.add_argument("--pose-limit", type=int, default=30)
    comparison.set_defaults(handler=create_comparison)

    finalize = subparsers.add_parser(
        "finalize-comparison", help="비교 결과 metadata 완성"
    )
    finalize.add_argument("csv")
    finalize.set_defaults(handler=finalize_comparison)

    activate = subparsers.add_parser("activate", help="run을 전체 시스템 활성값으로 선택")
    activate.add_argument("run")
    activate.set_defaults(handler=activate_run)

    active = subparsers.add_parser("show-active", help="현재 활성 run 출력")
    active.set_defaults(handler=show_active)

    values = subparsers.add_parser("values", help="static TF용 xyz quaternion 출력")
    values.add_argument("selector", nargs="?", default="active")
    values.set_defaults(handler=active_values)

    resolve = subparsers.add_parser("resolve", help="run 또는 calib 선택자를 경로로 변환")
    resolve.add_argument("selector")
    resolve.set_defaults(handler=resolve_command)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    try:
        return int(arguments.handler(arguments))
    except (FileNotFoundError, ValueError, KeyError, RuntimeError) as error:
        print(f"오류: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
