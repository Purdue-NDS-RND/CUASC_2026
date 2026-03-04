from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    default_config = (
        get_package_share_directory("drone_demo") + "/config/mission_square.yaml"
    )

    config = LaunchConfiguration("config")

    takeoff_service_node = Node(
        package="drone_demo",
        executable="simple_takeoff_service",
        name="simple_takeoff_service",
        output="screen",
        parameters=[config],
    )

    mission_node = Node(
        package="drone_demo",
        executable="waypoint_demo_mission",
        name="waypoint_demo_mission",
        output="screen",
        parameters=[config, {"config_file": config}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to mission YAML config",
            ),
            takeoff_service_node,
            mission_node,
        ]
    )