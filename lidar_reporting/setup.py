import os
from setuptools import setup

package_name = 'lidar_reporting'

setup(
    name=package_name,
    version='0.0.0',
    # Explicitly declaration guarantees all submodules are bundled into the install space
    packages=[
        'lidar_reporting',
        'lidar_reporting.nodes',
        'lidar_reporting.tools',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aarush',
    maintainer_email='aarush@polymathrobotics.com',
    description='Polymath Robotics LiDAR evaluation reporting node',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'metrics_reporting_node = lidar_reporting.nodes.metrics_reporting_node:main',
            'visualizer_node = lidar_reporting.nodes.visualizer_node:main',
        ],
    },
)
