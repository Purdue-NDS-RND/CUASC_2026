from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    spawn_model = LaunchConfiguration("spawn_model")
    spawn_service = LaunchConfiguration("spawn_service")
    radius_m = LaunchConfiguration("radius_m")
    hover_radius_m = LaunchConfiguration("hover_radius_m")
    hover_duration_s = LaunchConfiguration("hover_duration_s")
    target_altitude_m = LaunchConfiguration("target_altitude_m")

    spawner = Node(
        package="drone_control",
        executable="target_spawner",
        name="target_spawner",
        output="screen",
        parameters=[
            {"spawn_model": spawn_model},
            {"spawn_service": spawn_service},
            {"radius_m": radius_m},
            {"hover_radius_m": hover_radius_m},
            {"hover_duration_s": hover_duration_s},
            {"target_altitude_m": target_altitude_m},
        ],
    )

    follower = Node(
        package="drone_control",
        executable="target_follower",
        name="target_follower",
        output="screen",
        parameters=[
            {"hover_altitude_m": target_altitude_m},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spawn_model", default_value="true", description="Spawn a Gazebo model."
            ),
            DeclareLaunchArgument(
                "spawn_service",
                default_value="/world/map/create",
                description="Gazebo spawn service.",
            ),
            DeclareLaunchArgument(
                "radius_m", default_value="30.0", description="Spawn radius in meters."
            ),
            DeclareLaunchArgument(
                "hover_radius_m", default_value="2.0", description="Hover radius in meters."
            ),
            DeclareLaunchArgument(
                "hover_duration_s",
                default_value="5.0",
                description="Hover duration before respawn.",
            ),
            DeclareLaunchArgument(
                "target_altitude_m",
                default_value="20.0",
                description="Target hover altitude above the object.",
            ),
            spawner,
            follower,
        ]
    )
