import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    # 1. Find the path to your YAML file
    config_path = os.path.join(
        get_package_share_directory('vision_pipeline'),
        'config',
        'vision_params.yaml'
    )

    # 2. Pass the config_path to the nodes
    image_grabber_node = Node(
        package="vision_pipeline",
        executable="image_grabber",
        name="image_grabber",
        output="screen",
        parameters=[config_path], # <--- Loaded here!
    )

    yolo_node = Node(
        package="vision_pipeline",
        executable="yolo_node",
        name="yolo_node",
        output="screen",
        parameters=[config_path], # <--- Loaded here!
    )

    return LaunchDescription(
        [
            image_grabber_node,
            yolo_node,
        ]
    )