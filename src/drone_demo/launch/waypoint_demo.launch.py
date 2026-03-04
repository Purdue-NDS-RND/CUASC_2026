from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("drone_demo")
    default_config = pkg_share + "/config/mission_square.yaml"
    default_params = pkg_share + "/config/mission_square_params.yaml"

    config = LaunchConfiguration("config")
    params = LaunchConfiguration("params")

    takeoff_service_node = Node(
        package="drone_demo",
        executable="simple_takeoff_service",
        name="simple_takeoff_service",
        output="screen",
        parameters=[params],
    )

    mission_node = Node(
        package="drone_demo",
        executable="waypoint_demo_mission",
        name="waypoint_demo_mission",
        output="screen",
        parameters=[params, {"config_file": config}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to mission YAML file (waypoints read directly by node)",
            ),
            DeclareLaunchArgument(
                "params",
                default_value=default_params,
                description="Path to ROS params YAML file (passed as --params-file)",
            ),
            takeoff_service_node,
            mission_node,
        ]
    )