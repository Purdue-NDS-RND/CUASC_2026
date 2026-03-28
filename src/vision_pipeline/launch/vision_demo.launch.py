import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 1. Find the path to your YAML file
    config_path = os.path.join(
        get_package_share_directory("vision_pipeline"), "config", "vision_params.yaml"
    )

    # 1. Load the URDF file
    urdf_path = os.path.join(
        get_package_share_directory("vision_pipeline"), "urdf", "hexacopter.urdf"
    )
    with open(urdf_path, "r") as infp:
        robot_desc = infp.read()

    # 2. Add the Robot State Publisher to your LaunchDescription
    urdf_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    # 2. Pass the config_path to the nodes
    image_grabber_node = Node(
        package="vision_pipeline",
        executable="image_grabber",
        name="image_grabber",
        output="screen",
        parameters=[config_path],  # <--- Loaded here!
    )

    yolo_node = Node(
        package="vision_pipeline",
        executable="yolo_node",
        name="yolo_node",
        output="screen",
        parameters=[config_path],  # <--- Loaded here!
    )

    simple_raycaster_node = (
        Node(
            package="vision_pipeline",
            executable="simple_raycaster",
            name="simple_raycaster",
            output="screen",
        ),
    )

    # 4. The Global GPS Geolocator
    target_geolocator_node = Node(
        package="vision_pipeline",
        executable="target_geolocator",
        name="target_geolocator",
        output="screen",
    )

    return LaunchDescription(
        [
            urdf_node,
            image_grabber_node,
            yolo_node,
            simple_raycaster_node,
            target_geolocator_node,
        ]
    )
