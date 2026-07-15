from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("settle_seconds", default_value="1.5"),
            DeclareLaunchArgument("detection_timeout_seconds", default_value="8.0"),
            Node(
                package="pinkk_handeye_automation",
                executable="auto_collect",
                name="pinkk_handeye_auto_collect",
                output="screen",
                parameters=[
                    {
                        "execute": LaunchConfiguration("execute"),
                        "settle_seconds": LaunchConfiguration("settle_seconds"),
                        "detection_timeout_seconds": LaunchConfiguration(
                            "detection_timeout_seconds"
                        ),
                    }
                ],
            ),
        ]
    )
