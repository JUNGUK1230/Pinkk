"""ROS Image에서 YOLO Pose를 실행해 USB 포트 keypoint 메시지를 발행한다."""

from __future__ import annotations

from pathlib import Path

from pinkk_usb_insertion_interfaces.msg import (
    Keypoint2D,
    UsbPortDetection,
    UsbPortDetectionArray,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

from .image_conversion import array_to_bgr8_image, bgr8_image_to_array
from .perception.yolo_adapter import normalize_yolo_pose


class YoloKeypointNode(Node):
    """학습 모델 경로를 파라미터로 받아 로봇 제어와 독립적으로 추론한다."""

    def __init__(self) -> None:
        """Pose 모델과 ROS 입출력 토픽을 준비한다."""
        super().__init__('pinkk_yolo_keypoint_node')
        
        self.declare_parameter('image_size', 640)
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('visibility_threshold', 0.01)
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('debug_image_enabled', True)

        package_path = Path(
            get_package_share_directory('pinkk_usb_insertion')
)

        model_path = (
            package_path /
        'models' /
        'usb_01.pt'
)
        if not model_path.is_file():
            raise FileNotFoundError(f'YOLO 모델을 찾을 수 없습니다: {model_path}')
        self._image_size = int(self.get_parameter('image_size').value)
        self._confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self._visibility_threshold = float(
            self.get_parameter('visibility_threshold').value
        )
        self._device = str(self.get_parameter('device').value)
        self._debug_enabled = bool(
            self.get_parameter('debug_image_enabled').value
        )
        if self._image_size <= 0:
            raise ValueError('image_size는 0보다 커야 합니다')

        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise RuntimeError(
                'ultralytics가 설치되지 않았습니다. YOLO 실행 환경을 확인하세요'
            ) from error
        self._model = YOLO(str(model_path))
        if str(self._model.task) != 'pose':
            raise ValueError(f'YOLO Pose 모델이 아닙니다: task={self._model.task}')
        keypoint_shape = getattr(self._model.model, 'kpt_shape', None)
        if not keypoint_shape or int(keypoint_shape[0]) != 4:
            raise ValueError(f'keypoint 4개 모델이 아닙니다: {keypoint_shape}')

        self._detection_publisher = self.create_publisher(
            UsbPortDetectionArray,
            '/robot_arm/perception/usb_port/detections',
            qos_profile_sensor_data,
        )
        self._debug_publisher = self.create_publisher(
            Image,
            '/robot_arm/perception/usb_port/debug_image',
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'YOLO Pose 준비: model={model_path}, device={self._device}, '
            f'imgsz={self._image_size}, conf={self._confidence_threshold:.2f}'
        )

    def _image_callback(self, message: Image) -> None:
        try:
            frame = bgr8_image_to_array(message)
            result = self._model.predict(
                frame,
                imgsz=self._image_size,
                conf=self._confidence_threshold,
                device=self._device,
                verbose=False,
            )[0]
            array = UsbPortDetectionArray()
            array.header = message.header
            if result.boxes is not None and result.keypoints is not None:
                point_confidence = result.keypoints.conf
                if point_confidence is None:
                    raise ValueError('모델이 keypoint confidence를 출력하지 않습니다')
                records = normalize_yolo_pose(
                    result.boxes.xywh.cpu().numpy(),
                    result.boxes.cls.cpu().numpy(),
                    result.boxes.conf.cpu().numpy(),
                    result.keypoints.xy.cpu().numpy(),
                    point_confidence.cpu().numpy(),
                    self._model.names,
                    self._visibility_threshold,
                )
                array.detections = [
                    self._to_message(record, index, message)
                    for index, record in enumerate(records)
                ]
            self._detection_publisher.publish(array)

            has_debug_subscriber = (
                self._debug_publisher.get_subscription_count() > 0
            )
            if self._debug_enabled and has_debug_subscriber:
                debug = array_to_bgr8_image(
                    result.plot(),
                    header=message.header,
                )
                self._debug_publisher.publish(debug)
        except Exception as error:
            self.get_logger().error(f'YOLO frame 처리 실패: {error}')

    @staticmethod
    def _to_message(record, index: int, image: Image) -> UsbPortDetection:
        detection = UsbPortDetection()
        detection.header = image.header
        detection.detection_id = (
            f'{image.header.stamp.sec}-{image.header.stamp.nanosec}-{index}'
        )
        detection.class_name = record.class_name
        detection.object_confidence = record.object_confidence
        detection.source_image_width = image.width
        detection.source_image_height = image.height
        detection.bbox.center.position.x = record.center_x
        detection.bbox.center.position.y = record.center_y
        detection.bbox.size_x = record.width
        detection.bbox.size_y = record.height
        detection.keypoints = [
            Keypoint2D(
                index=point.index,
                x=point.x,
                y=point.y,
                confidence=point.confidence,
                visible=point.visible,
            )
            for point in record.keypoints
        ]
        return detection


def main(args: list[str] | None = None) -> None:
    """YOLO keypoint 노드를 실행한다."""
    rclpy.init(args=args)
    node = YoloKeypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
