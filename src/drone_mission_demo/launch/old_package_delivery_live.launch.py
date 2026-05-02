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

    usb_grabber_node = Node(
        package="drone_target_cv",
        executable="usb_grabber",
        name="usb_grabber",
        output="screen",
        parameters=[params],
    )

    takeoff_service_node = Node(
        package="drone_utils",
        executable="simple_takeoff_service",
        name="simple_takeoff_service",
        output="screen",
        parameters=[params],
    )

    target_cv_node = Node(
        package="drone_target_cv",
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

    session_logger_node = Node(
        package="drone_utils",
        executable="session_logger",
        name="session_logger",
        output="screen",
        condition=IfCondition(LaunchConfiguration("log_session")),
        parameters=[params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value="config/params/old_live_target_mission.yaml",
                description="ROS params YAML relative to the drone_mission_demo package",
            ),
            DeclareLaunchArgument(
                "sequence",
                default_value="config/sequences/package_delivery_live.yaml",
                description="Mission sequence YAML relative to the drone_mission_demo package",
            ),
            DeclareLaunchArgument(
                "log_session",
                default_value="true",
                description="Start the mission session logger",
            ),
            usb_grabber_node,
            takeoff_service_node,
            target_cv_node,
            executor_node,
            session_logger_node,
        ]
    )
