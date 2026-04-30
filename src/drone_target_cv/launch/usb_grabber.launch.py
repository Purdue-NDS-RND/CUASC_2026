from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution(
        [
            FindPackageShare("drone_target_cv"),
            LaunchConfiguration("params"),
        ]
    )

    usb_grabber_node = Node(
        package="drone_target_cv",
        executable="usb_grabber",
        name="usb_grabber",
        output="screen",
        parameters=[
            params,
            {
                "camera_type": LaunchConfiguration("camera_type"),
                "device_path": LaunchConfiguration("device_path"),
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "publish_width": LaunchConfiguration("publish_width"),
                "publish_height": LaunchConfiguration("publish_height"),
                "fps": LaunchConfiguration("fps"),
                "image_publishing_rate": LaunchConfiguration("image_publishing_rate"),
                "frame_id": LaunchConfiguration("frame_id"),
                "publish_raw": LaunchConfiguration("publish_raw"),
                "publish_compressed": LaunchConfiguration("publish_compressed"),
                "compressed_quality": LaunchConfiguration("compressed_quality"),
                "reset_v4l2_controls": LaunchConfiguration("reset_v4l2_controls"),
                "lock_white_balance": LaunchConfiguration("lock_white_balance"),
                "manual_white_balance": LaunchConfiguration("manual_white_balance"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value="config/usb_grabber_live.yaml",
                description="ROS params YAML relative to the drone_target_cv package",
            ),
            # Options: rolling, global. device_path below overrides this selection when set.
            DeclareLaunchArgument(
                "camera_type",
                default_value="global",
                description="Camera type preset: rolling or global",
            ),
            DeclareLaunchArgument(
                "device_path",
                default_value="",
                description="Optional stable Linux camera path override like /dev/v4l/by-id/...",
            ),
            DeclareLaunchArgument(
                "image_width",
                default_value="640",
                description="Requested USB camera image width in pixels",
            ),
            DeclareLaunchArgument(
                "image_height",
                default_value="480",
                description="Requested USB camera image height in pixels",
            ),
            DeclareLaunchArgument(
                "publish_width",
                default_value="640",
                description="Published image width in pixels after resize",
            ),
            DeclareLaunchArgument(
                "publish_height",
                default_value="480",
                description="Published image height in pixels after resize",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="60.0",
                description="Requested USB camera frame rate",
            ),
            DeclareLaunchArgument(
                "image_publishing_rate",
                default_value="30.0",
                description="ROS image publish rate in Hz",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="camera_link",
                description="Frame id for published Image and CameraInfo messages",
            ),
            DeclareLaunchArgument(
                "publish_raw",
                default_value="false",
                description="Publish raw sensor_msgs/Image on /camera/image",
            ),
            DeclareLaunchArgument(
                "publish_compressed",
                default_value="true",
                description="Publish JPEG-compressed frames on /camera/image/compressed",
            ),
            DeclareLaunchArgument(
                "compressed_quality",
                default_value="20",
                description="JPEG quality used for /camera/image/compressed",
            ),
            DeclareLaunchArgument(
                "reset_v4l2_controls",
                default_value="true",
                description="Reset global-camera V4L2 image controls to their defaults before white balance",
            ),
            DeclareLaunchArgument(
                "lock_white_balance",
                default_value="true",
                description="Disable automatic white balance and use manual_white_balance",
            ),
            DeclareLaunchArgument(
                "manual_white_balance",
                default_value="4500",
                description="Manual white balance temperature for V4L2/OpenCV controls",
            ),
            usb_grabber_node,
        ]
    )
