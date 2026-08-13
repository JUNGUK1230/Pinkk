"""로봇 PC trajectory bridge를 사용하는 노트북 MoveIt 실행 구성."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _include(filename: str, **launch_arguments: str) -> IncludeLaunchDescription:
    launch_dir = Path(get_package_share_directory("mycobot_280_moveit2")) / "launch"
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / filename)),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description() -> LaunchDescription:
    # ros2_control/fake hardware를 띄우지 않는다. 실제 trajectory action은
    # 로봇 PC의 pinkk_mycobot_trajectory_bridge가 제공한다.
    return LaunchDescription(
        [
            _include("static_virtual_joint_tfs.launch.py"),
            _include("rsp.launch.py"),
            _include("move_group.launch.py", allow_trajectory_execution="true"),
            _include("moveit_rviz.launch.py"),
        ]
    )
