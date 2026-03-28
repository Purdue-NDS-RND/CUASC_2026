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
        # Using glob automatically grabs all files in these directories
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
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
            # The new unified localizer replaces the old raycaster/geolocator
            "target_localizer = vision_pipeline.target_localizer:main",
        ],
    },
)
