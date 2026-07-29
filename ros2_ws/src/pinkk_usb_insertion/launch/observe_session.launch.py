"""로봇 bridge를 시작하고 초기 관측 자세로 한 번 복귀한다."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    insertion_share = Path(
        get_package_share_directory('pinkk_usb_insertion')
    )
    bridge_share = Path(
        get_package_share_directory('pinkk_mycobot_bridge')
    )
    runtime = str(insertion_share / 'config' / 'hybrid_runtime.yaml')
    bridge_config = str(
        bridge_share / 'config' / 'trajectory_bridge.yaml'
    )
    bridge = Node(
        package='pinkk_mycobot_bridge',
        executable='trajectory_bridge',
        name='pinkk_mycobot_trajectory_bridge',
        output='screen',
        parameters=[bridge_config],
    )
    return_to_observe = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='pinkk_usb_insertion',
                executable='return_to_observe',
                name='pinkk_return_to_observe',
                output='screen',
                parameters=[runtime],
            )
        ],
    )
    return LaunchDescription([bridge, return_to_observe])
