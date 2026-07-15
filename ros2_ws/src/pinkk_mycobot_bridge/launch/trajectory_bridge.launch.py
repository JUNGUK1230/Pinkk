from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud", default_value="1000000"),
            DeclareLaunchArgument("speed", default_value="10"),
            DeclareLaunchArgument("goal_tolerance_deg", default_value="2.0"),
            Node(
                package="pinkk_mycobot_bridge",
                executable="trajectory_bridge",
                name="pinkk_mycobot_trajectory_bridge",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "baud": LaunchConfiguration("baud"),
                        "speed": LaunchConfiguration("speed"),
                        "goal_tolerance_deg": LaunchConfiguration("goal_tolerance_deg"),
                        "publish_rate_hz": 10.0,
                        "max_execution_seconds": 60.0,
                    }
                ],
            ),
        ]
    )
