"""YAML 한 파일로 PyMyCobot 실행 bridge를 시작한다."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('pinkk_mycobot_bridge'))
    config = str(share / 'config' / 'trajectory_bridge.yaml')
    return LaunchDescription(
        [
            Node(
                package='pinkk_mycobot_bridge',
                executable='trajectory_bridge',
                name='pinkk_mycobot_trajectory_bridge',
                output='screen',
                parameters=[config],
            )
        ]
    )
