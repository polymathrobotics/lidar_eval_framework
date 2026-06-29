import os
from setuptools import find_packages, setup

package_name = 'arduino_eth_bridge'


def package_files(data_files, directory):
    for path, _, filenames in os.walk(directory):
        install_path = os.path.join('share', package_name, path)
        files = [os.path.join(path, f) for f in filenames]
        if files:
            data_files.append((install_path, files))
    return data_files


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

data_files = package_files(data_files, 'config')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Polymath Robotics Engineering',
    maintainer_email='engineering@polymathrobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            "arduino_eth_bridge = arduino_eth_bridge.arduino_eth_bridge_node:main",
            "arduino_eth_bridge_node = arduino_eth_bridge.arduino_eth_bridge_node:main",
        ],
    },
)
