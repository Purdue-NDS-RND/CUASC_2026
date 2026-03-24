from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution([
        FindPackageShare("drone_package_drop"),
        LaunchConfiguration("params"),
    ])

    takeoff_service_node = Node(
        package="drone_utils",
        executable="simple_takeoff_service",
        name="simple_takeoff_service",
        output="screen",
        parameters=[params],
    )

    gimbal_service_node = Node(
        package="drone_utils",
        executable="gimbal_point_service",
        name="gimbal_point_service",
        output="screen",
    )

    payload_drop_node = Node(
        package="drone_package_drop",
        executable="payload_drop",
        name="payload_drop",
        output="screen",
        parameters=[params],
    )

    target_cv_node = Node(
        package="drone_package_drop",
        executable="target_cv",
        name="target_cv",
        output="screen",
        parameters=[params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value="config/mission_params.yaml",
                description="Path to ROS params YAML file relative to package share",
            ),
            takeoff_service_node,
            gimbal_service_node,
            payload_drop_node,
            target_cv_node,
        ]
    )
