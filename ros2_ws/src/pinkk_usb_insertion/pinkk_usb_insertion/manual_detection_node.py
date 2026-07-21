"""수동 네 점을 YOLO와 같은 검출 메시지로 변환하는 개발용 노드."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import numpy as np
from pinkk_usb_insertion_interfaces.msg import (
    Keypoint2D,
    UsbPortDetection,
    UsbPortDetectionArray,
)
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray

from .configuration import load_yaml


def _default_config(filename: str) -> str:
    return str(Path(get_package_share_directory('pinkk_usb_insertion')) / 'config' / filename)


class ManualDetectionNode(Node):
    """운영 YOLO 노드 대신 테스트 검출을 발행하며 실제 운용에는 사용하지 않는다."""

    def __init__(self) -> None:
        super().__init__('pinkk_manual_detection_node')
        self.declare_parameter('camera_config', _default_config('camera_intrinsics.yaml'))
        camera = load_yaml(str(self.get_parameter('camera_config').value))['camera']
        self._frame_id = str(camera['frame_id'])
        self._width = int(camera['image_width'])
        self._height = int(camera['image_height'])
        self._camera_matrix = [
            float(value) for row in camera['camera_matrix'] for value in row
        ]
        self._distortion = [float(value) for value in camera['distortion_coefficients']]
        self._detection_publisher = self.create_publisher(
            UsbPortDetectionArray,
            '/robot_arm/perception/usb_port/detections',
            10,
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo,
            '/camera/camera_info',
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            '/robot_arm/perception/usb_port/manual_keypoints',
            self._keypoint_callback,
            10,
        )
        self.create_timer(0.5, self._publish_camera_info)

    def _make_camera_info(self) -> CameraInfo:
        camera_info = CameraInfo()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = self._frame_id
        camera_info.width = self._width
        camera_info.height = self._height
        camera_info.distortion_model = 'plumb_bob'
        camera_info.k = self._camera_matrix
        camera_info.d = self._distortion
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [
            self._camera_matrix[0], self._camera_matrix[1], self._camera_matrix[2], 0.0,
            self._camera_matrix[3], self._camera_matrix[4], self._camera_matrix[5], 0.0,
            self._camera_matrix[6], self._camera_matrix[7], self._camera_matrix[8], 0.0,
        ]
        return camera_info

    def _publish_camera_info(self) -> None:
        self._camera_info_publisher.publish(self._make_camera_info())

    def _keypoint_callback(self, message: Float64MultiArray) -> None:
        if len(message.data) != 8:
            self.get_logger().error('수동 keypoint는 총 8개 값이어야 합니다')
            return
        points = np.asarray(message.data, dtype=np.float64).reshape(4, 2)
        if not np.all(np.isfinite(points)):
            self.get_logger().error('수동 keypoint에 NaN 또는 inf가 있습니다')
            return

        camera_info = self._make_camera_info()
        self._camera_info_publisher.publish(camera_info)

        detection = UsbPortDetection()
        detection.header = camera_info.header
        detection.detection_id = 'manual-0'
        detection.class_name = 'usb_port'
        detection.object_confidence = 1.0
        detection.source_image_width = self._width
        detection.source_image_height = self._height
        detection.bbox.center.position.x = float(np.mean(points[:, 0]))
        detection.bbox.center.position.y = float(np.mean(points[:, 1]))
        detection.bbox.size_x = float(np.ptp(points[:, 0]))
        detection.bbox.size_y = float(np.ptp(points[:, 1]))
        detection.keypoints = [
            Keypoint2D(
                index=index,
                x=float(point[0]),
                y=float(point[1]),
                confidence=1.0,
                visible=True,
            )
            for index, point in enumerate(points)
        ]

        array = UsbPortDetectionArray()
        array.header = detection.header
        array.detections = [detection]
        self._detection_publisher.publish(array)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ManualDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
