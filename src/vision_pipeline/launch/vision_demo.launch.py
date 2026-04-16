import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 1. Find the path to your YAML file for parameters
    config_path = os.path.join(
        get_package_share_directory("vision_pipeline"), "config", "vision_params.yaml"
    )

    # 2. Load the URDF file for the camera mount kinematics
    urdf_path = os.path.join(
        get_package_share_directory("vision_pipeline"), "urdf", "hexacopter.urdf"
    )
    with open(urdf_path, "r") as infp:
        robot_desc = infp.read()

    # 3. Add the Robot State Publisher to broadcast the URDF tree
    urdf_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    # 4. Hardware driver for the Arducam
    image_grabber_node = Node(
        package="vision_pipeline",
        executable="image_grabber",
        name="image_grabber",
        output="screen",
        parameters=[config_path],
    )

    # 5. TensorRT YOLO Inference Engine
    yolo_node = Node(
        package="vision_pipeline",
        executable="yolo_node",
        name="yolo_node",
        output="screen",
        parameters=[config_path],
    )

    # 6. The New Mission Logger (Replaces raycaster & geolocator)
    mission_logger_node = Node(
        package="vision_pipeline",
        executable="mission_logger",
        name="mission_logger",
        output="screen",
        parameters=[config_path],  # Passes ground_altitude_m from your yaml
    )

    return LaunchDescription(
        [
            urdf_node,
            image_grabber_node,
            yolo_node,
            mission_logger_node,
        ]
    )
