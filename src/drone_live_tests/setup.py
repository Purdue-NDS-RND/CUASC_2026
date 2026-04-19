from glob import glob

from setuptools import find_packages, setup


package_name = "drone_live_tests"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config/sequences", glob("config/sequences/*.yaml")),
        ("share/" + package_name + "/config/params", glob("config/params/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="ppatel",
    maintainer_email="pate2293@purdue.edu",
    description="Live flight landing-sequence test missions built on drone_mission_core.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
)
