from setuptools import find_packages, setup

package_name = 'lidar_transforms'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/lidar_bench.urdf']),
        ('share/' + package_name + '/config', ['config/roi.yaml', 'config/lidar_frame.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aarush',
    maintainer_email='aarush@polymathrobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_bench_tf_broadcaster = lidar_transforms.lidar_bench_tf_broadcaster:main',
            'lidar_baseline_node = lidar_transforms.lidar_baseline_node:main',
        ],
    },
)
