from setuptools import find_packages, setup

package_name = 'drone_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ppatel',
    maintainer_email='pate2293@purdue.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_takeoff_service = drone_utils.simple_takeoff_service:main',
            'gimbal_point_service = drone_utils.gimble_point_service:main',
            'session_logger = drone_utils.session_logger:main',
        ],
    },
)
