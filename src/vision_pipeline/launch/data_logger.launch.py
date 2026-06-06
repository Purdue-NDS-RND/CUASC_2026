import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Image Grabber Node Configuration
    image_grabber_node = Node(
        package='vision_pipeline',
        executable='image_grabber',
        name='image_grabber',
        parameters=[{
            'image_width': 3840,
            'image_height': 2160,
            'fps': 17,
            'image_publishing_rate': 4.0,
            'camera_info_file': 'arducam_info.yaml',
            'enable_timelapse': False # Disabled here since data_logger handles saving
        }],
        output='screen'
    )

    # 2. Background Data Logger Node Configuration
    data_logger_node = Node(
        package='vision_pipeline',
        executable='data_logger_node',
        name='background_data_logger',
        output='screen'
    )

    return LaunchDescription([
        image_grabber_node,
        data_logger_node
    ])
