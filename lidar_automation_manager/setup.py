import glob

from setuptools import find_packages, setup

package_name = 'lidar_automation_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/automation_manager.yaml']),
        ('share/' + package_name + '/config/lidar_driver_files',
            glob.glob('config/lidar_driver_files/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aarush',
    maintainer_email='aarush@polymathrobotics.com',
    description='Automates lidar test-bench runs: drives drivers, angles, parameter sweeps and bag recording.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'automation_manager = lidar_automation_manager.nodes.automation_manager_node:main',
        ],
    },
)
