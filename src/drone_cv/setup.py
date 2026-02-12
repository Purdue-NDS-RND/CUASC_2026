from setuptools import setup

package_name = "drone_control"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/target_demo.launch.py",
            "launch/waypoint_demo.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ppatel",
    maintainer_email="pate2293@purdue.edu",
    description="Target spawner and follower nodes for MAVROS-based control.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "target_spawner = drone_control.target_spawner:main",
            "waypoint_follower = drone_control.waypoint_follower:main",
            "target_localizer = drone_control.target_localizer:main",
            "target_visualizer = drone_control.target_visualizer:main",
            "detection_simulator = drone_control.detection_simulator:main",
            "simple_takeoff = drone_control.simple_takeoff:main",
        ],
    },
)
