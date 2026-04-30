from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    image_topic = LaunchConfiguration("image_topic")
    params = PathJoinSubstitution(
        [
            FindPackageShare("drone_target_cv"),
            LaunchConfiguration("params"),
        ]
    )

    image_grabber_node = Node(
        package="drone_target_cv",
        executable="compressed_grabber",
        name="image_grabber",
        output="screen",
        remappings=[
            ("/camera/image_raw/compressed", [image_topic]),
        ],
        parameters=[params],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value="config/compressed_grabber_demo.yaml",
                description="ROS params YAML relative to the drone_target_cv package",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image",
                description="Output topic for the published camera stream",
            ),
            image_grabber_node,
        ]
    )
