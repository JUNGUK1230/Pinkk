"""로봇 bridge를 시작하고 초기 관측 자세로 한 번 복귀한다."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    insertion_share = Path(
        get_package_share_directory('pinkk_usb_insertion')
    )
    bridge_share = Path(
        get_package_share_directory('pinkk_mycobot_bridge')
    )
    runtime = str(insertion_share / 'config' / 'hybrid_runtime.yaml')
    profile = LaunchConfiguration('robot_profile')
    profile_runtime = PathJoinSubstitution(
        [
            str(insertion_share),
            'config',
            'robots',
            profile,
            'runtime.yaml',
        ]
    )
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
                parameters=[runtime, profile_runtime],
            )
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_profile',
                default_value='robot_a',
                choices=['robot_a', 'robot_b'],
                description='사용할 로봇별 관측 자세·파라미터 프로필',
            ),
            bridge,
            return_to_observe,
        ]
    )
