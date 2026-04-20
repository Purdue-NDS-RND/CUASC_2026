from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    image_topic = LaunchConfiguration("image_topic")

    image_grabber_node = Node(
        package="vision_pipeline",
        executable="image_grabber",
        name="image_grabber",
        output="screen",
        remappings=[("/camera/image_raw", image_topic)],
        parameters=[
            {
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "fps": LaunchConfiguration("fps"),
                "image_publishing_rate": LaunchConfiguration("image_publishing_rate"),
                "publish_full_res": LaunchConfiguration("publish_full_res"),
                "monitor_width": LaunchConfiguration("monitor_width"),
                "monitor_height": LaunchConfiguration("monitor_height"),
                "shutter_speed": LaunchConfiguration("shutter_speed"),
                "wb_mode": LaunchConfiguration("wb_mode"),
                "camera_info_file": LaunchConfiguration("camera_info_file"),
                "enable_timelapse": LaunchConfiguration("enable_timelapse"),
                "save_dir": LaunchConfiguration("save_dir"),
                "save_interval_sec": LaunchConfiguration("save_interval_sec"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image",
                description="Output topic for the published camera stream",
            ),
            DeclareLaunchArgument(
                "image_width",
                default_value="1280",
                description="Requested camera image width in pixels",
            ),
            DeclareLaunchArgument(
                "image_height",
                default_value="720",
                description="Requested camera image height in pixels",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="15",
                description="Camera sensor frame rate",
            ),
            DeclareLaunchArgument(
                "image_publishing_rate",
                default_value="10.0",
                description="ROS image publish rate in Hz",
            ),
            DeclareLaunchArgument(
                "publish_full_res",
                default_value="false",
                description="Publish the full-resolution frame on the main image topic",
            ),
            DeclareLaunchArgument(
                "monitor_width",
                default_value="960",
                description="Width of the low-latency published stream",
            ),
            DeclareLaunchArgument(
                "monitor_height",
                default_value="540",
                description="Height of the low-latency published stream",
            ),
            DeclareLaunchArgument(
                "shutter_speed",
                default_value="1000",
                description="Camera shutter speed parameter",
            ),
            DeclareLaunchArgument(
                "wb_mode",
                default_value="6",
                description="Camera white-balance mode",
            ),
            DeclareLaunchArgument(
                "camera_info_file",
                default_value="arducam_info.yaml",
                description="Camera calibration YAML in vision_pipeline/config",
            ),
            DeclareLaunchArgument(
                "enable_timelapse",
                default_value="false",
                description="Save frames to disk while streaming",
            ),
            DeclareLaunchArgument(
                "save_dir",
                default_value="/tmp/camera_captures_calibrated",
                description="Directory used when timelapse saving is enabled",
            ),
            DeclareLaunchArgument(
                "save_interval_sec",
                default_value="1.0",
                description="Seconds between saved timelapse frames",
            ),
            image_grabber_node,
        ]
    )
