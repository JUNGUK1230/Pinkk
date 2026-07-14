from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("camera", default_value="0"),
        DeclareLaunchArgument(
            "intrinsics_path",
            default_value=(
                "/home/jetcobot/Pinkk-robot-arm/src/robot_arm/robot_camera/"
                "camera_calibration/results/intrinsics.npz"
            ),
        ),
        DeclareLaunchArgument("show_preview", default_value="true"),
    ]
    node = Node(
        package="pinkk_mycobot_bridge",
        executable="charuco_tf_publisher",
        name="pinkk_charuco_tf_publisher",
        output="screen",
        parameters=[
            {
                "camera": LaunchConfiguration("camera"),
                "intrinsics_path": LaunchConfiguration("intrinsics_path"),
                "show_preview": LaunchConfiguration("show_preview"),
                "camera_frame": "camera_optical_frame",
                "target_frame": "charuco_board",
                "camera_width": 640,
                "camera_height": 480,
                "min_corners": 35,
                "max_reprojection_error_px": 0.7,
            }
        ],
    )
    return LaunchDescription([*arguments, node])
