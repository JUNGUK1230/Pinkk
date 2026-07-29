"""기존 통합 bridge를 사용해 초기 관측 관절 자세로 한 번 복귀한다."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('pinkk_usb_insertion'))
    runtime = str(share / 'config' / 'hybrid_runtime.yaml')
    return LaunchDescription(
        [
            Node(
                package='pinkk_usb_insertion',
                executable='return_to_observe',
                name='pinkk_return_to_observe',
                output='screen',
                parameters=[runtime],
            )
        ]
    )
