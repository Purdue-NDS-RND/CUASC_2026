from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config = PathJoinSubstitution([
        FindPackageShare("drone_demo"),
        LaunchConfiguration("config")
    ])
    params = PathJoinSubstitution([
        FindPackageShare("drone_demo"),
        LaunchConfiguration("params")
    ])

    takeoff_service_node = Node(
        package="drone_utils",
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
                default_value="config/waypoint_square.yaml",
                description="Path to mission YAML file relative to package share (waypoints read directly by node)",
            ),
            DeclareLaunchArgument(
                "params",
                default_value="config/mission_params.yaml",
                description="Path to ROS params YAML file relative to package share (passed as --params-file)",
            ),
            takeoff_service_node,
            mission_node,
        ]
    )