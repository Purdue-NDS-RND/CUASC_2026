import os
from glob import glob

from setuptools import find_packages, setup

package_name = "vision_pipeline"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),  # Better practice than [package_name]
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Tell ROS to copy your launch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        # Tell ROS to copy your yaml files
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        # Tell ROS to copy your YOLO engine
        (
            os.path.join("share", package_name, "models"),
            glob(os.path.join("models", "*")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.urdf")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="samuel_yoon",
    maintainer_email="yoon315@purdue.edu",
    description="Vision pipeline (grabs images, runs inference, localizes targets)",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "image_grabber = vision_pipeline.image_grabber:main",
            "yolo_node = vision_pipeline.yolo_node:main",
            "mission_logger = vision_pipeline.mission_logger:main",
        ],
    },
)
