"""Flask MJPEG에서 USB 네 모서리를 클릭해 T_camera_usb TF를 발행한다."""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import requests
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster

from ..config import settings as config
from ..core.charuco import load_intrinsics


POINT_LABELS = ("좌상단", "우상단", "우하단", "좌하단")


class MjpegStream:
    """MJPEG 최신 frame을 별도 스레드에서 수신한다."""

    def __init__(self, url: str) -> None:
        self.url = url
        self.latest: np.ndarray | None = None
        self.lock = threading.Lock()
        self.active = False
        self.thread: threading.Thread | None = None
        self.error: Exception | None = None

    def start(self) -> None:
        self.active = True
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.active = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)

    def frame(self) -> np.ndarray | None:
        with self.lock:
            return None if self.latest is None else self.latest.copy()

    def _receive(self) -> None:
        try:
            with requests.get(self.url, stream=True, timeout=(5, 10)) as response:
                response.raise_for_status()
                buffer = bytearray()
                for chunk in response.iter_content(chunk_size=4096):
                    if not self.active:
                        break
                    buffer.extend(chunk)
                    start = buffer.find(b"\xff\xd8")
                    end = buffer.find(b"\xff\xd9", start + 2)
                    if start < 0 or end < 0:
                        continue
                    encoded = bytes(buffer[start : end + 2])
                    del buffer[: end + 2]
                    frame = cv2.imdecode(
                        np.frombuffer(encoded, np.uint8),
                        cv2.IMREAD_COLOR,
                    )
                    if frame is not None:
                        with self.lock:
                            self.latest = frame
        except Exception as error:
            self.error = error
        finally:
            self.active = False


def estimate_usb_pose(
    points_px: list[tuple[float, float]],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    width_mm: float,
    height_mm: float,
) -> tuple[np.ndarray, np.ndarray, float, np.ndarray]:
    """네 모서리로 USB-to-camera rvec/tvec와 재투영 오차를 구한다."""
    if len(points_px) != 4:
        raise ValueError("USB 모서리 네 점이 필요합니다")
    half_width = width_mm / 2.0
    half_height = height_mm / 2.0
    object_points = np.array(
        [
            [-half_width, -half_height, 0.0],
            [half_width, -half_height, 0.0],
            [half_width, half_height, 0.0],
            [-half_width, half_height, 0.0],
        ],
        dtype=np.float64,
    )
    image_points = np.asarray(points_px, dtype=np.float64)
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE,
    )
    if not success or float(tvec[2, 0]) <= 0.0:
        raise RuntimeError("solvePnP가 올바른 USB pose를 계산하지 못했습니다")
    projected, _ = cv2.projectPoints(
        object_points,
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs,
    )
    projected = projected.reshape(-1, 2)
    error = float(np.mean(np.linalg.norm(projected - image_points, axis=1)))
    return rvec, tvec, error, projected


class ManualUsbTf:
    """마우스 클릭 상태와 마지막 T_camera_usb TF를 관리한다."""

    def __init__(
        self,
        node: rclpy.node.Node,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        width_mm: float,
        height_mm: float,
        camera_frame: str,
        usb_frame: str,
    ) -> None:
        self.node = node
        self.broadcaster = TransformBroadcaster(node)
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.width_mm = width_mm
        self.height_mm = height_mm
        self.camera_frame = camera_frame
        self.usb_frame = usb_frame
        self.frozen: np.ndarray | None = None
        self.points: list[tuple[float, float]] = []
        self.pose: tuple[np.ndarray, np.ndarray, float, np.ndarray] | None = None

    def reset(self) -> None:
        self.points.clear()
        self.pose = None

    def mouse(self, event: int, x: int, y: int, _flags: int, _data: object) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or self.frozen is None:
            return
        if len(self.points) >= 4:
            return
        self.points.append((float(x), float(y)))
        print(f"{len(self.points)}번 {POINT_LABELS[len(self.points)-1]}: ({x}, {y})")
        if len(self.points) == 4:
            self.pose = estimate_usb_pose(
                self.points,
                self.camera_matrix,
                self.dist_coeffs,
                self.width_mm,
                self.height_mm,
            )
            _rvec, tvec, error, _projected = self.pose
            xyz = tvec.reshape(3)
            print(
                f"T_camera_usb [mm]: X={xyz[0]:.2f}, Y={xyz[1]:.2f}, "
                f"Z={xyz[2]:.2f}, reprojection={error:.3f}px"
            )
            print(f"TF 발행: {self.camera_frame} -> {self.usb_frame}")

    def publish(self) -> None:
        if self.pose is None:
            return
        rvec, tvec, _error, _projected = self.pose
        matrix, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = Rotation.from_matrix(matrix).as_quat()
        x, y, z = tvec.reshape(3) / 1000.0
        message = TransformStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = self.camera_frame
        message.child_frame_id = self.usb_frame
        message.transform.translation.x = float(x)
        message.transform.translation.y = float(y)
        message.transform.translation.z = float(z)
        message.transform.rotation.x = float(qx)
        message.transform.rotation.y = float(qy)
        message.transform.rotation.z = float(qz)
        message.transform.rotation.w = float(qw)
        self.broadcaster.sendTransform(message)

    def draw(self, live: np.ndarray) -> np.ndarray:
        view = (self.frozen if self.frozen is not None else live).copy()
        for index, point in enumerate(self.points):
            pixel = tuple(np.rint(point).astype(int))
            cv2.circle(view, pixel, 5, (0, 255, 255), -1)
            cv2.putText(
                view,
                str(index + 1),
                (pixel[0] + 6, pixel[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )
        if len(self.points) > 1:
            polygon = np.rint(self.points).astype(np.int32)
            cv2.polylines(view, [polygon], len(self.points) == 4, (0, 255, 255), 1)
        if self.pose is not None:
            rvec, tvec, error, projected = self.pose
            cv2.drawFrameAxes(
                view,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                10.0,
                2,
            )
            for point in projected:
                cv2.circle(view, tuple(np.rint(point).astype(int)), 3, (255, 0, 255), -1)
            xyz = tvec.reshape(3)
            cv2.putText(
                view,
                f"camera USB: {xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f} mm err {error:.2f}px",
                (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
        mode = "FROZEN" if self.frozen is not None else "LIVE"
        cv2.putText(
            view,
            f"{mode} | f freeze/live | r reset | q quit",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 0),
            1,
        )
        return view


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://192.168.6.1:5000/stream")
    parser.add_argument("--calib", type=Path, default=config.INTRINSICS_PATH)
    parser.add_argument("--usb-width", type=float, default=11.5)
    parser.add_argument("--usb-height", type=float, default=4.5)
    parser.add_argument("--camera-frame", default="camera_optical_frame")
    parser.add_argument("--usb-frame", default="usb_port")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    camera_matrix, dist_coeffs, expected_size = load_intrinsics(args.calib)
    rclpy.init()
    node = rclpy.create_node("manual_usb_tf_publisher")
    app = ManualUsbTf(
        node,
        camera_matrix,
        dist_coeffs,
        args.usb_width,
        args.usb_height,
        args.camera_frame,
        args.usb_frame,
    )
    stream = MjpegStream(args.url)
    window = "manual_usb_tf"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, app.mouse)
    stream.start()
    size_checked = False
    try:
        while rclpy.ok():
            frame = stream.frame()
            if frame is None:
                if stream.error is not None:
                    raise RuntimeError(f"MJPEG 수신 실패: {stream.error}")
                time.sleep(0.05)
                continue
            if not size_checked:
                actual = (frame.shape[1], frame.shape[0])
                if expected_size is not None and actual != expected_size:
                    raise RuntimeError(
                        f"영상 해상도 {actual}와 내부 캘리브레이션 {expected_size}가 다릅니다"
                    )
                print(f"영상 해상도 확인: {actual[0]} x {actual[1]}")
                size_checked = True
            app.publish()
            rclpy.spin_once(node, timeout_sec=0.0)
            cv2.imshow(window, app.draw(frame))
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("f"):
                if app.frozen is None:
                    app.frozen = frame.copy()
                    app.reset()
                    print("화면 고정: 좌상단부터 네 모서리를 클릭하세요")
                else:
                    app.frozen = None
                    app.reset()
            elif key == ord("r"):
                app.reset()
            time.sleep(0.01)
    finally:
        stream.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

