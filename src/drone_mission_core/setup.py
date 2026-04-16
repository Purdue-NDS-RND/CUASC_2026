from setuptools import find_packages, setup


package_name = "drone_mission_core"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="ppatel",
    maintainer_email="pate2293@purdue.edu",
    description="Reusable timer-driven mission framework for ROS2 drone missions.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mission_executor = drone_mission_core.mission_executor:main",
        ],
    },
)
