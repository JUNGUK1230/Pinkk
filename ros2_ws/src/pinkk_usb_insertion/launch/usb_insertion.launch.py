from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('pinkk_usb_insertion'))
    config = share / 'config'
    common_control = str(config / 'insertion_control.yaml')
    return LaunchDescription(
        [
            DeclareLaunchArgument('use_manual_input', default_value='false'),
            Node(
                package='pinkk_usb_insertion',
                executable='manual_detection_node',
                name='pinkk_manual_detection_node',
                output='screen',
                condition=IfCondition(LaunchConfiguration('use_manual_input')),
                parameters=[
                    {'camera_config': str(config / 'camera_intrinsics.yaml')}
                ],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='port_pose_node',
                name='pinkk_port_pose_node',
                output='screen',
                parameters=[{'control_config': common_control}],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='arm_motion_node',
                name='pinkk_arm_motion_node',
                output='screen',
                parameters=[
                    {
                        'control_config': common_control,
                        'tool_config': str(config / 'tool_transform.yaml'),
                    }
                ],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='usb_insertion_node',
                name='pinkk_usb_insertion_node',
                output='screen',
                parameters=[{'control_config': common_control}],
            ),
        ]
    )
