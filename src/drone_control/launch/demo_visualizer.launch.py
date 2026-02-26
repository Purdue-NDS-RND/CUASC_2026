"""
Demo Visualizer Launch File

Launches the visualization demo with noisy GPS tracking:
  1. waypoint_demo - simulation + drone control + target spawning
  2. target_tracker_sim - adds distance-based noise to target GPS
  3. target_visualizer - shows drone, true target, and noisy estimates

Usage:
  ros2 launch drone_control demo_visualizer.launch.py
  ros2 launch drone_control demo_visualizer.launch.py target_type:=bw_target
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    # Get package directory
    pkg_dir = get_package_share_directory("drone_control")
    
    # Launch arguments
    target_type = LaunchConfiguration("target_type")
    max_noise_m = LaunchConfiguration("max_noise_m")
    min_noise_m = LaunchConfiguration("min_noise_m")
    use_noisy_gps = LaunchConfiguration("use_noisy_gps")

    # Include waypoint_demo launch
    waypoint_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "waypoint_demo.launch.py")
        ),
        launch_arguments={
            "target_type": target_type,
            "use_noisy_gps": use_noisy_gps,
        }.items(),
    )

    # Target tracker simulator - adds noise to GPS
    tracker_node = Node(
        package="drone_control",
        executable="target_tracker_sim",
        name="target_tracker_sim",
        output="screen",
        parameters=[
            {"update_rate_hz": 10.0},
            {"max_noise_m": max_noise_m},
            {"min_noise_m": min_noise_m},
            {"noise_falloff_dist_m": 60.0},
            {"detection_range_m": 100.0},
        ],
    )

    # Target visualizer
    visualizer_node = Node(
        package="drone_control",
        executable="target_visualizer",
        name="target_visualizer",
        output="screen",
        parameters=[
            {"update_rate_hz": 10.0},
            {"history_size": 100},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "target_type",
                default_value="bw_target",
                description="Target type: 'red_box' or 'bw_target'",
            ),
            DeclareLaunchArgument(
                "max_noise_m",
                default_value="8.0",
                description="Maximum GPS noise (meters) when far from target",
            ),
            DeclareLaunchArgument(
                "min_noise_m",
                default_value="0.5",
                description="Minimum GPS noise (meters) when close to target",
            ),
            DeclareLaunchArgument(
                "use_noisy_gps",
                default_value="true",
                description="Use noisy GPS data for waypoint navigation (simulates real sensor)",
            ),
            waypoint_demo,
            tracker_node,
            visualizer_node,
        ]
    )
