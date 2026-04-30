import os
from glob import glob

from setuptools import find_packages, setup


package_name = "drone_target_cv"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="ppatel",
    maintainer_email="pate2293@purdue.edu",
    description="Target CV nodes split out from vision_pipeline for mission demos.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "target_cv = drone_target_cv.target_cv:main",
            "usb_grabber = drone_target_cv.usb_grabber:main",
            "debug_viewer = drone_target_cv.debug_viewer:main",
        ],
    },
)
