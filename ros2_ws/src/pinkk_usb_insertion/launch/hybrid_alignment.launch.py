"""실기용 perception, PBVS 계산과 제조사 API 실행기를 한 번에 시작한다."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    return LaunchDescription(
        [
            _robot_description_launch('static_virtual_joint_tfs.launch.py'),
            _robot_description_launch('rsp.launch.py'),
            Node(
                package='pinkk_usb_insertion',
                executable='camera_publisher_node',
                name='pinkk_usb_camera_node',
                output='screen',
                parameters=[runtime],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='yolo_keypoint_node',
                name='pinkk_yolo_keypoint_node',
                output='screen',
                parameters=[runtime],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='port_pose_node',
                name='pinkk_port_pose_node',
                output='screen',
                parameters=[runtime],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='pbvs_alignment_node',
                name='pinkk_pbvs_alignment_node',
                output='screen',
                parameters=[runtime],
            ),
            Node(
                package='pinkk_usb_insertion',
                executable='pbvs_step_executor_node',
                name='pinkk_pbvs_step_executor_node',
                output='screen',
                parameters=[runtime],
            ),
        ]
    )
