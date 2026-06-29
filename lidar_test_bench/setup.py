import os
from setuptools import find_packages, setup

package_name = 'lidar_test_bench'


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

# Install config/ and launch/ folders
data_files = package_files(data_files, 'config')
data_files = package_files(data_files, 'launch')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='willnickerson',
    maintainer_email='willrnickerson@polymathrobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_node = lidar_test_bench.nodes.lidar_node:main',
            'lidar_controller = lidar_test_bench.nodes.lidar_controller:main',
        ],
    },
)
