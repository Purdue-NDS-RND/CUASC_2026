from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params = PathJoinSubstitution(
        [
            FindPackageShare("drone_live_tests"),
            LaunchConfiguration("params"),
        ]
    )
    sequence = PathJoinSubstitution(
        [
            FindPackageShare("drone_live_tests"),
            LaunchConfiguration("sequence"),
        ]
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
                "mission_modules": ["drone_live_tests.missions"],
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value="config/params/live_landing_test_params.yaml",
                description="ROS params YAML relative to the drone_live_tests package",
            ),
            DeclareLaunchArgument(
                "sequence",
                default_value="config/sequences/live_landing_test.yaml",
                description="Mission sequence YAML relative to the drone_live_tests package",
            ),
            executor_node,
        ]
    )
