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
        parameters=[params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value="config/usb_grabber_live.yaml",
                description="ROS params YAML relative to the drone_target_cv package",
            ),
            usb_grabber_node,
        ]
    )
