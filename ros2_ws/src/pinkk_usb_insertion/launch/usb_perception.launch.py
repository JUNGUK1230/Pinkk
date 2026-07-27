"""USB 카메라, YOLO Pose와 solvePnP만 실행하는 비제어 launch."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """로봇 제어 없이 카메라·YOLO·solvePnP 노드만 구성한다."""
    share = Path(get_package_share_directory('pinkk_usb_insertion'))
    config = share / 'config'
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'camera_device', default_value='/dev/video2'
            ),
            DeclareLaunchArgument(
                'model_path', default_value='/home/juwon/Desktop/usb_01.pt'
            ),
            DeclareLaunchArgument('inference_device', default_value='cpu'),
            DeclareLaunchArgument(
                'debug_image_enabled', default_value='true'
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='camera_publisher_node',
                name='pinkk_usb_camera_node',
                output='screen',
                parameters=[
                    {
                        'camera_device': LaunchConfiguration('camera_device'),
                        'camera_config': str(
                            config / 'camera_intrinsics.yaml'
                        ),
                    }
                ],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='yolo_keypoint_node',
                name='pinkk_yolo_keypoint_node',
                output='screen',
                parameters=[
                    {
                        'model_path': LaunchConfiguration('model_path'),
                        'device': LaunchConfiguration('inference_device'),
                        'debug_image_enabled': ParameterValue(
                            LaunchConfiguration('debug_image_enabled'),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='port_pose_node',
                name='pinkk_port_pose_node',
                output='screen',
                parameters=[
                    {'control_config': str(config / 'insertion_control.yaml')}
                ],
            ),
        ]
    )
