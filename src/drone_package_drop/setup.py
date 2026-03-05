from setuptools import setup

package_name = "drone_package_drop"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/payload_drop.launch.py"]),
        ("share/" + package_name + "/config", ["config/mission_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ppatel",
    maintainer_email="pate2293@purdue.edu",
    description="Precision payload drop mission using GPS navigation and visual target tracking.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "payload_drop = drone_package_drop.payload_drop:main",
            "target_cv = drone_package_drop.target_cv:main",
        ],
    },
)
