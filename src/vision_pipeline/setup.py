import os
from glob import glob
from setuptools import find_packages, setup

package_name = "vision_pipeline"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
            ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            ('share/' + package_name + '/launch', ['launch/vision_demo.launch.py']),
            ('share/' + package_name + '/config', ['config/vision_params.yaml']), 
            (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="samuel_yoon",
    maintainer_email="yoon315@purdue.edu",
    description="Vision pipeline (grabs images and runs inference on them)",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "image_grabber = vision_pipeline.image_grabber:main",
            "yolo_node = vision_pipeline.yolo_node:main",
        ],
    },
)
