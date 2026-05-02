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

    mipi_grabber_node = Node(
        package="drone_target_cv",
        executable="mipi_grabber",
        name="mipi_grabber",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value="config/mipi_params.yaml",
                description="ROS params YAML relative to the drone_target_cv package",
            ),
            mipi_grabber_node,
        ]
    )
