"""기존 통합 bridge를 사용해 초기 관측 관절 자세로 한 번 복귀한다."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('pinkk_usb_insertion'))
    runtime = str(share / 'config' / 'hybrid_runtime.yaml')
    profile = LaunchConfiguration('robot_profile')
    profile_runtime = PathJoinSubstitution(
        [str(share), 'config', 'robots', profile, 'runtime.yaml']
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_profile',
                default_value='robot_a',
                choices=['robot_a', 'robot_b'],
                description='사용할 로봇별 관측 자세·파라미터 프로필',
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='return_to_observe',
                name='pinkk_return_to_observe',
                output='screen',
                parameters=[runtime, profile_runtime],
            )
        ]
    )
