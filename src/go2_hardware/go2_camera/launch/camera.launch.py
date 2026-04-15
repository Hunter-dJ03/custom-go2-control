"""Launch the Go2 front camera node with configurable parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "network_interface",
                default_value="enp2s0",
                description="Network interface connected to Go2 MCU",
            ),
            DeclareLaunchArgument(
                "target_fps",
                default_value="30",
                description="Target publishing frame rate",
            ),
            DeclareLaunchArgument(
                "jpeg_quality",
                default_value="80",
                description="JPEG compression quality (0-100) for compressed topic",
            ),
            Node(
                package="go2_camera",
                executable="camera_node",
                name="go2_camera_node",
                namespace="/go2/front_camera",
                parameters=[
                    {
                        "network_interface": LaunchConfiguration("network_interface"),
                        "target_fps": LaunchConfiguration("target_fps"),
                        "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                    }
                ],
                remappings=[
                    ("~/image_raw", "/go2/front_camera/image_raw"),
                    ("~/image_raw/compressed", "/go2/front_camera/image_raw/compressed"),
                ],
                output="screen",
            ),
        ]
    )
