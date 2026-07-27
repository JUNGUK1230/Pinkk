"""노트북 USB 카메라 영상과 보정된 CameraInfo를 ROS 토픽으로 발행한다."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .configuration import load_yaml
from .image_conversion import array_to_bgr8_image


def _default_config(filename: str) -> str:
    share = Path(get_package_share_directory('pinkk_usb_insertion'))
    return str(share / 'config' / filename)


class CameraPublisherNode(Node):
    """지정한 V4L2 장치를 캘리브레이션 해상도로 고정해 발행한다."""

    def __init__(self) -> None:
        """카메라·보정 설정을 검증하고 영상 발행 타이머를 시작한다."""
        super().__init__('pinkk_usb_camera_node')
        self.declare_parameter('camera_device', '/dev/video2')
        self.declare_parameter(
            'camera_config', _default_config('camera_intrinsics.yaml')
        )
        self.declare_parameter('publish_rate_hz', 30.0)

        config_path = str(self.get_parameter('camera_config').value)
        camera = load_yaml(config_path)['camera']
        self._device = str(self.get_parameter('camera_device').value)
        self._frame_id = str(camera['frame_id'])
        self._width = int(camera['image_width'])
        self._height = int(camera['image_height'])
        self._camera_matrix = [
            float(value) for row in camera['camera_matrix'] for value in row
        ]
        self._distortion = [
            float(value) for value in camera['distortion_coefficients']
        ]
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate_hz는 0보다 커야 합니다')

        self._capture = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        if not self._capture.isOpened():
            raise RuntimeError(f'USB 카메라를 열 수 없습니다: {self._device}')
        ok, frame = self._capture.read()
        if not ok:
            self._capture.release()
            raise RuntimeError(f'USB 카메라 첫 프레임 수신 실패: {self._device}')
        if frame.shape[1] != self._width or frame.shape[0] != self._height:
            self._capture.release()
            raise RuntimeError(
                '카메라 해상도가 내부 보정과 다릅니다: '
                f'actual={frame.shape[1]}x{frame.shape[0]}, '
                f'calibrated={self._width}x{self._height}'
            )

        self._image_publisher = self.create_publisher(
            Image, '/camera/image_raw', qos_profile_sensor_data
        )
        self._info_publisher = self.create_publisher(
            CameraInfo, '/camera/camera_info', qos_profile_sensor_data
        )
        self.create_timer(1.0 / publish_rate, self._publish_frame)
        self.get_logger().info(
            f'USB 카메라 발행 시작: {self._device}, '
            f'{self._width}x{self._height}, {publish_rate:.1f}Hz'
        )

    def _camera_info(self) -> CameraInfo:
        message = CameraInfo()
        message.width = self._width
        message.height = self._height
        message.distortion_model = 'plumb_bob'
        message.k = self._camera_matrix
        message.d = self._distortion
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [
            self._camera_matrix[0],
            self._camera_matrix[1],
            self._camera_matrix[2],
            0.0,
            self._camera_matrix[3],
            self._camera_matrix[4],
            self._camera_matrix[5],
            0.0,
            self._camera_matrix[6],
            self._camera_matrix[7],
            self._camera_matrix[8],
            0.0,
        ]
        return message

    def _publish_frame(self) -> None:
        ok, frame = self._capture.read()
        if not ok:
            self.get_logger().error('USB 카메라 프레임 수신 실패')
            return
        if frame.shape[:2] != (self._height, self._width):
            self.get_logger().error(
                '내부 보정과 다른 해상도 프레임을 거부합니다: '
                f'{frame.shape[1]}x{frame.shape[0]}'
            )
            return

        stamp = self.get_clock().now().to_msg()
        image = array_to_bgr8_image(frame)
        image.header.stamp = stamp
        image.header.frame_id = self._frame_id
        info = self._camera_info()
        info.header = image.header
        self._image_publisher.publish(image)
        self._info_publisher.publish(info)

    def destroy_node(self) -> bool:
        """카메라 장치를 해제한 뒤 ROS 노드를 종료한다."""
        if hasattr(self, '_capture'):
            self._capture.release()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    """USB 카메라 publisher 노드를 실행한다."""
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
