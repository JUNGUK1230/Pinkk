from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("execute", default_value="false"),
            DeclareLaunchArgument("old_calib_path"),
            DeclareLaunchArgument("new_calib_path"),
            DeclareLaunchArgument("output_csv", default_value=""),
            DeclareLaunchArgument("pose_limit", default_value="30"),
            DeclareLaunchArgument("measurement_count", default_value="10"),
            DeclareLaunchArgument("settle_seconds", default_value="1.5"),
            DeclareLaunchArgument("detection_timeout_seconds", default_value="8.0"),
            Node(
                package="pinkk_handeye_automation",
                executable="compare_calibrations",
                name="pinkk_handeye_compare_calibrations",
                output="screen",
                parameters=[
                    {
                        "execute": LaunchConfiguration("execute"),
                        "old_calib_path": LaunchConfiguration("old_calib_path"),
                        "new_calib_path": LaunchConfiguration("new_calib_path"),
                        "output_csv": LaunchConfiguration("output_csv"),
                        "pose_limit": LaunchConfiguration("pose_limit"),
                        "measurement_count": LaunchConfiguration("measurement_count"),
                        "settle_seconds": LaunchConfiguration("settle_seconds"),
                        "detection_timeout_seconds": LaunchConfiguration(
                            "detection_timeout_seconds"
                        ),
                    }
                ],
            ),
        ]
    )
