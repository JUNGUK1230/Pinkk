from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baud", default_value="1000000"),
            DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
            Node(
                package="pinkk_mycobot_bridge",
                executable="joint_state_publisher",
                name="pinkk_mycobot_joint_state_publisher",
                output="screen",
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "baud": LaunchConfiguration("baud"),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                    }
                ],
            ),
        ]
    )
