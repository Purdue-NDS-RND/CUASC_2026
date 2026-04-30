from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    target_cv_node = Node(
        package="drone_target_cv",
        executable="target_cv",
        name="target_cv",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "compressed_input": LaunchConfiguration("compressed_input"),
                "debug_view": LaunchConfiguration("debug_view"),
                "start_enabled": LaunchConfiguration("start_enabled"),
                "sim_hsv": LaunchConfiguration("sim_hsv"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="camera/image/compressed",
                description="Camera image topic consumed by target_cv",
            ),
            DeclareLaunchArgument(
                "compressed_input",
                default_value="true",
                description="Subscribe to sensor_msgs/CompressedImage instead of raw Image",
            ),
            DeclareLaunchArgument(
                "debug_view",
                default_value="true",
                description="Publish target_cv annotated and mask debug streams",
            ),
            DeclareLaunchArgument(
                "start_enabled",
                default_value="true",
                description="Start target detection immediately on launch",
            ),
            DeclareLaunchArgument(
                "sim_hsv",
                default_value="false",
                description="Use sim red HSV thresholds when true; live/outdoor thresholds when false",
            ),
            target_cv_node,
        ]
    )
