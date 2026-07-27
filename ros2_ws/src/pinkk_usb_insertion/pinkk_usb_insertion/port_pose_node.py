"""YOLO keypoint 토픽을 구독해 카메라 기준 USB 포트 pose를 계산한다."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import numpy as np
from pinkk_usb_insertion_interfaces.msg import (
    UsbPortDetectionArray,
    UsbPortObservation,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

from .configuration import load_yaml
from .perception.detection_selector import select_detection
from .perception.pose_estimator import estimate_port_pose
from .ros_utils import transform_to_pose


def _default_config(filename: str) -> str:
    return str(Path(get_package_share_directory('pinkk_usb_insertion')) / 'config' / filename)


class PortPoseNode(Node):
    """YOLO 검출을 선택·검증하고 solvePnP 결과를 한 메시지로 발행한다."""

    def __init__(self) -> None:
        super().__init__('pinkk_port_pose_node')
        self.declare_parameter('control_config', _default_config('insertion_control.yaml'))
        control = load_yaml(str(self.get_parameter('control_config').value))
        model = control['port_model']
        limits = control['pose_estimation']
        self._port_width = float(model['width_m'])
        self._port_height = float(model['height_m'])
        self._minimum_depth = float(limits['minimum_depth_m'])
        self._maximum_depth = float(limits['maximum_depth_m'])
        self._maximum_error = float(limits['maximum_reprojection_error_px'])
        self._minimum_object_confidence = float(limits['minimum_object_confidence'])
        self._minimum_keypoint_confidence = float(limits['minimum_keypoint_confidence'])
        self._target_class_name = str(limits['target_class_name'])
        self._target_detection_id = str(limits['target_detection_id'])
        self._maximum_detection_age = float(
            control['safety']['maximum_detection_age_seconds']
        )
        self._camera_info: CameraInfo | None = None

        self._observation_publisher = self.create_publisher(
            UsbPortObservation,
            '/robot_arm/perception/usb_port/observation',
            10,
        )
        self._pose_publisher = self.create_publisher(
            PoseStamped,
            '/robot_arm/perception/usb_port/pose_camera',
            10,
        )
        self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            UsbPortDetectionArray,
            '/robot_arm/perception/usb_port/detections',
            self._detection_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info('YOLO USB keypoint 검출과 CameraInfo를 기다립니다')

    def _camera_info_callback(self, message: CameraInfo) -> None:
        if message.width > 0 and message.height > 0 and len(message.k) == 9:
            self._camera_info = message

    def _invalid_observation(
        self,
        message: UsbPortDetectionArray,
        reason: str,
    ) -> None:
        observation = UsbPortObservation()
        observation.header = message.header
        observation.valid = False
        observation.rejection_reason = reason
        self._observation_publisher.publish(observation)
        self.get_logger().warning(reason, throttle_duration_sec=2.0)

    def _detection_callback(self, message: UsbPortDetectionArray) -> None:
        if self._camera_info is None:
            self._invalid_observation(message, 'CameraInfo를 아직 받지 못했습니다')
            return
        try:
            stamp_seconds = (
                float(message.header.stamp.sec)
                + float(message.header.stamp.nanosec) * 1e-9
            )
            now_seconds = self.get_clock().now().nanoseconds * 1e-9
            age_seconds = now_seconds - stamp_seconds
            if stamp_seconds <= 0.0 or not 0.0 <= age_seconds <= self._maximum_detection_age:
                raise ValueError(f'YOLO 검출 시간이 유효하지 않습니다: age={age_seconds:.3f}s')
            selected = select_detection(
                message.detections,
                self._minimum_object_confidence,
                self._minimum_keypoint_confidence,
                self._target_class_name,
                self._target_detection_id,
            )
            detection = selected.detection
            if (
                int(detection.source_image_width) != int(self._camera_info.width)
                or int(detection.source_image_height) != int(self._camera_info.height)
            ):
                raise ValueError('YOLO 원본 영상과 CameraInfo 해상도가 다릅니다')
            if detection.header.frame_id != self._camera_info.header.frame_id:
                raise ValueError('YOLO 검출과 CameraInfo frame_id가 다릅니다')

            camera_matrix = np.asarray(self._camera_info.k, dtype=np.float64).reshape(3, 3)
            distortion = np.asarray(self._camera_info.d, dtype=np.float64)
            estimate = estimate_port_pose(
                selected.ordered_points_px,
                camera_matrix,
                distortion,
                self._port_width,
                self._port_height,
            )
            if not self._minimum_depth <= estimate.depth_m <= self._maximum_depth:
                raise ValueError(f'포트 깊이가 허용 범위를 벗어났습니다: {estimate.depth_m:.4f}m')
            if estimate.reprojection_error_px > self._maximum_error:
                raise ValueError(
                    f'재투영 오차가 기준을 초과했습니다: '
                    f'{estimate.reprojection_error_px:.3f}px'
                )
        except (ValueError, RuntimeError) as error:
            self._invalid_observation(message, f'포트 자세 추정 거부: {error}')
            return

        observation = UsbPortObservation()
        observation.header = detection.header
        observation.detection_id = detection.detection_id
        observation.pose = transform_to_pose(estimate.camera_to_port)
        observation.keypoints = detection.keypoints
        observation.object_confidence = detection.object_confidence
        observation.reprojection_error_px = estimate.reprojection_error_px
        observation.depth_m = estimate.depth_m
        observation.valid = True
        observation.rejection_reason = ''
        self._observation_publisher.publish(observation)

        pose = PoseStamped()
        pose.header = observation.header
        pose.pose = observation.pose
        self._pose_publisher.publish(pose)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PortPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
