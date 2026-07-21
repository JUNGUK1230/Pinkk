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
            DeclareLaunchArgument(
                "cartesian_max_translation_m", default_value="0.0105"
            ),
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
                        "cartesian_base_frame": "g_base",
                        "cartesian_position_tolerance_m": 0.001,
                        "cartesian_orientation_tolerance_deg": 1.0,
                        "cartesian_max_translation_m": LaunchConfiguration(
                            "cartesian_max_translation_m"
                        ),
                        "cartesian_max_rotation_deg": 2.1,
                        "cartesian_path_z_tolerance_m": 0.002,
                        "cartesian_path_tilt_tolerance_deg": 3.0,
                        "cartesian_timeout_seconds": 15.0,
                    }
                ],
            ),
        ]
    )
