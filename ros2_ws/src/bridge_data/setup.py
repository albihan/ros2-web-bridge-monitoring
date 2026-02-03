from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'bridge_data'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Alfareza Giovani',
    maintainer_email='rzgvn11@gmail.com',

    description=(
        'ROS 2 data bridge for real-time sensor data integration using '
        'micro-ROS, rosbridge, and web-based visualization dashboard'
    ),

    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'iot_bridge = bridge_data.data_publisher:main',
        ],
    },
)