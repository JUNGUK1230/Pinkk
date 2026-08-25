"""Perception/PBVS와 초기 관측 고정목표 시험 실행기를 시작한다."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _robot_description_launch(filename: str) -> IncludeLaunchDescription:
    launch_dir = (
        Path(get_package_share_directory('mycobot_280_moveit2')) / 'launch'
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / filename))
    )


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory('pinkk_usb_insertion'))
    runtime = str(share / 'config' / 'hybrid_runtime.yaml')
    profile = LaunchConfiguration('robot_profile')
    model_path = LaunchConfiguration('model_path')
    profile_root = PathJoinSubstitution(
        [str(share), 'config', 'robots', profile]
    )
    profile_runtime = PathJoinSubstitution([profile_root, 'runtime.yaml'])
    camera_config = PathJoinSubstitution(
        [profile_root, 'camera_intrinsics.yaml']
    )
    handeye_config = PathJoinSubstitution([profile_root, 'handeye.yaml'])
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'robot_profile',
                default_value='robot_a',
                choices=['robot_a', 'robot_b'],
                description='사용할 로봇별 보정·파라미터 프로필',
            ),
            DeclareLaunchArgument(
                'model_path',
                default_value='models/usb_02.pt',
                description='YOLO Pose weight 경로',
            ),
            _robot_description_launch('static_virtual_joint_tfs.launch.py'),
            _robot_description_launch('rsp.launch.py'),
            Node(
                package='pinkk_usb_insertion',
                executable='camera_publisher_node',
                name='pinkk_usb_camera_node',
                output='screen',
                parameters=[
                    runtime,
                    profile_runtime,
                    {'camera_config': camera_config},
                ],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='yolo_keypoint_node',
                name='pinkk_yolo_keypoint_node',
                output='screen',
                parameters=[
                    runtime,
                    profile_runtime,
                    {'model_path': model_path},
                ],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='port_pose_node',
                name='pinkk_port_pose_node',
                output='screen',
                parameters=[runtime, profile_runtime],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='pbvs_alignment_node',
                name='pinkk_pbvs_alignment_node',
                output='screen',
                parameters=[
                    runtime,
                    profile_runtime,
                    {'handeye_config': handeye_config},
                ],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='frozen_target_executor_node',
                name='pinkk_frozen_target_executor_node',
                output='screen',
                parameters=[runtime, profile_runtime],
            ),
        ]
    )
