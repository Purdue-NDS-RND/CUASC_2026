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

    target_cv_node = Node(
        package="drone_target_cv",
        executable="target_cv",
        name="target_cv",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value="config/target_cv_live.yaml",
                description="ROS params YAML relative to the drone_target_cv package",
            ),
            target_cv_node,
        ]
    )
