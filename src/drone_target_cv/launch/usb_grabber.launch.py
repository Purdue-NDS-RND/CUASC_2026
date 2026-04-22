from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    usb_grabber_node = Node(
        package="drone_target_cv",
        executable="usb_grabber",
        name="usb_grabber",
        output="screen",
        parameters=[
            {
                "device_index": LaunchConfiguration("device_index"),
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "fps": LaunchConfiguration("fps"),
                "image_publishing_rate": LaunchConfiguration("image_publishing_rate"),
                "frame_id": LaunchConfiguration("frame_id"),
                "camera_info_file": LaunchConfiguration("camera_info_file"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device_index",
                default_value="0",
                description="OpenCV USB camera device index",
            ),
            DeclareLaunchArgument(
                "image_width",
                default_value="1280",
                description="Requested USB camera image width in pixels",
            ),
            DeclareLaunchArgument(
                "image_height",
                default_value="720",
                description="Requested USB camera image height in pixels",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="30.0",
                description="Requested USB camera frame rate",
            ),
            DeclareLaunchArgument(
                "image_publishing_rate",
                default_value="15.0",
                description="ROS image publish rate in Hz",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="camera_link",
                description="Frame id for published Image and CameraInfo messages",
            ),
            DeclareLaunchArgument(
                "camera_info_file",
                default_value="",
                description="Optional calibration YAML in drone_target_cv/config",
            ),
            usb_grabber_node,
        ]
    )
