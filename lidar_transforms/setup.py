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
        ('share/' + package_name + '/launch', ['launch/robot_description.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aarush',
    maintainer_email='aarush@polymathrobotics.com',
    description='Bench TF: URDF placement, robot_state_publisher launch, the motor-joint '
                'TF broadcaster, and the raw transform math shared across the bench.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_bench_tf_broadcaster = lidar_transforms.lidar_bench_tf_broadcaster:main',
        ],
    },
)
