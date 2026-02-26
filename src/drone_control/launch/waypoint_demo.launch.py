"""
Waypoint Demo Launch File

Launches the complete waypoint following demo:
  1. simple_takeoff - provides takeoff service
  2. target_spawner - spawns targets and publishes noisy positions  
  3. waypoint_follower - calls takeoff service and flies through waypoints

The drone will:
  - Take off to the specified altitude (via simple_takeoff service)
  - Fly to spawned targets using noisy position estimates
  - Hover briefly at each waypoint
  - Request a new waypoint (old one deleted, new one spawned)
  - Repeat until max_waypoints or shutdown

Usage:
  ros2 launch drone_control waypoint_demo.launch.py
  ros2 launch drone_control waypoint_demo.launch.py takeoff_altitude_m:=30.0 max_waypoints:=5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Launch arguments
    takeoff_altitude_m = LaunchConfiguration("takeoff_altitude_m")
    waypoint_altitude_m = LaunchConfiguration("waypoint_altitude_m")
    spawn_radius_m = LaunchConfiguration("spawn_radius_m")
    arrival_radius_m = LaunchConfiguration("arrival_radius_m")
    hover_time_s = LaunchConfiguration("hover_time_s")
    max_waypoints = LaunchConfiguration("max_waypoints")
    return_to_launch = LaunchConfiguration("return_to_launch")
    world_name = LaunchConfiguration("world_name")
    use_noisy_position = LaunchConfiguration("use_noisy_position")
    max_noise_m = LaunchConfiguration("max_noise_m")
    min_noise_m = LaunchConfiguration("min_noise_m")
    noise_falloff_dist_m = LaunchConfiguration("noise_falloff_dist_m")
    target_type = LaunchConfiguration("target_type")
    use_noisy_gps = LaunchConfiguration("use_noisy_gps")

    # Target spawner - spawns objects and publishes positions
    spawner_node = Node(
        package="drone_control",
        executable="target_spawner",
        name="target_spawner",
        output="screen",
        parameters=[
            {"spawn_model": True},
            {"world_name": world_name},
            {"radius_m": spawn_radius_m},
            {"target_altitude_m": waypoint_altitude_m},
            {"hover_duration_s": 999999.0},  # Disable auto-respawn, use service instead
            {"simulate_accuracy": True},
            {"max_noise_m": max_noise_m},
            {"min_noise_m": min_noise_m},
            {"noise_falloff_dist_m": noise_falloff_dist_m},
            {"target_type": target_type},
        ],
    )

    # Simple takeoff - provides /drone_control/takeoff service
    takeoff_node = Node(
        package="drone_control",
        executable="simple_takeoff",
        name="simple_takeoff",
        output="screen",
    )

    # Waypoint follower - main control node (calls simple_takeoff service)
    follower_node = Node(
        package="drone_control",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[
            {"takeoff_altitude_m": takeoff_altitude_m},
            {"waypoint_altitude_m": waypoint_altitude_m},
            {"arrival_radius_m": arrival_radius_m},
            {"hover_time_s": hover_time_s},
            {"max_waypoints": max_waypoints},
            {"return_to_launch": return_to_launch},
            {"use_noisy_position": use_noisy_position},
            {"use_noisy_gps": use_noisy_gps},
        ],
    )

    return LaunchDescription(
        [
            # Declare all arguments
            DeclareLaunchArgument(
                "takeoff_altitude_m",
                default_value="20.0",
                description="Initial takeoff altitude in meters",
            ),
            DeclareLaunchArgument(
                "waypoint_altitude_m",
                default_value="20.0",
                description="Altitude to fly above each waypoint",
            ),
            DeclareLaunchArgument(
                "spawn_radius_m",
                default_value="50.0",
                description="Radius around origin to spawn targets",
            ),
            DeclareLaunchArgument(
                "arrival_radius_m",
                default_value="5.0",
                description="Horizontal distance to consider 'arrived' at waypoint",
            ),
            DeclareLaunchArgument(
                "hover_time_s",
                default_value="3.0",
                description="Time to hover at each waypoint before requesting next",
            ),
            DeclareLaunchArgument(
                "max_waypoints",
                default_value="0",
                description="Maximum waypoints to visit (0 = unlimited)",
            ),
            DeclareLaunchArgument(
                "return_to_launch",
                default_value="false",
                description="Return to launch position after max waypoints",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="map",
                description="Gazebo world name for spawning",
            ),
            DeclareLaunchArgument(
                "use_noisy_position",
                default_value="true",
                description="Use noisy position estimates (simulates real sensors)",
            ),
            DeclareLaunchArgument(
                "max_noise_m",
                default_value="5.0",
                description="Maximum position noise when far from target",
            ),
            DeclareLaunchArgument(
                "min_noise_m",
                default_value="0.1",
                description="Minimum position noise when close to target",
            ),
            DeclareLaunchArgument(
                "noise_falloff_dist_m",
                default_value="50.0",
                description="Distance at which noise is maximum",
            ),
            DeclareLaunchArgument(
                "target_type",
                default_value="bw_target",
                description="Target type: 'red_box' or 'bw_target' (black-white GCP marker)",
            ),
            DeclareLaunchArgument(
                "use_noisy_gps",
                default_value="false",
                description="Use noisy GPS for navigation (subscribe to /drone_control/waypoint/gps_noisy)",
            ),
            # Nodes
            takeoff_node,
            spawner_node,
            follower_node,
        ]
    )
