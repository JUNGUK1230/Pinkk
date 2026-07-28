from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud", default_value="1000000"),
            DeclareLaunchArgument("speed", default_value="10"),
            DeclareLaunchArgument("goal_tolerance_deg", default_value="2.0"),
            DeclareLaunchArgument(
                "joint_execution_enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "joint_max_command_attempts", default_value="1"
            ),
            DeclareLaunchArgument(
                "cartesian_execution_enabled", default_value="false"
            ),
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
                        "goal_tolerance_deg": LaunchConfiguration(
                            "goal_tolerance_deg"
                        ),
                        "joint_execution_enabled": ParameterValue(
                            LaunchConfiguration("joint_execution_enabled"),
                            value_type=bool,
                        ),
                        "joint_stable_sample_count": 5,
                        "joint_stable_delta_deg": 0.2,
                        "joint_hold_check_seconds": 2.0,
                        "joint_max_command_attempts": ParameterValue(
                            LaunchConfiguration(
                                "joint_max_command_attempts"
                            ),
                            value_type=int,
                        ),
                        "joint_retry_stable_sample_count": 3,
                        "joint_retry_minimum_progress_deg": 0.1,
                        "publish_rate_hz": 10.0,
                        "max_execution_seconds": 60.0,
                        "cartesian_execution_enabled": ParameterValue(
                            LaunchConfiguration("cartesian_execution_enabled"),
                            value_type=bool,
                        ),
                        "cartesian_base_frame": "g_base",
                        "cartesian_position_tolerance_m": 0.0005,
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
