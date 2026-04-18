from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution(
        [
            FindPackageShare("drone_mission_demo"),
            LaunchConfiguration("params"),
        ]
    )
    sequence = PathJoinSubstitution(
        [
            FindPackageShare("drone_mission_demo"),
            LaunchConfiguration("sequence"),
        ]
    )

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
        condition=IfCondition(LaunchConfiguration("start_gimbal_service")),
    )

    target_cv_node = Node(
        package="vision_pipeline",
        executable="target_cv",
        name="target_cv",
        output="screen",
        parameters=[params],
    )

    executor_node = Node(
        package="drone_mission_core",
        executable="mission_executor",
        name="mission_executor",
        output="screen",
        parameters=[
            params,
            {
                "sequence_file": sequence,
                "mission_modules": ["drone_mission_demo.missions"],
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value="config/params/package_drop_params.yaml",
                description="ROS params YAML relative to the drone_mission_demo package",
            ),
            DeclareLaunchArgument(
                "sequence",
                default_value="config/sequences/package_drop_demo.yaml",
                description="Mission sequence YAML relative to the drone_mission_demo package",
            ),
            DeclareLaunchArgument(
                "start_gimbal_service",
                default_value="false",
                description="Start the gimbal proxy service for active gimbal control",
            ),
            takeoff_service_node,
            gimbal_service_node,
            target_cv_node,
            executor_node,
        ]
    )
