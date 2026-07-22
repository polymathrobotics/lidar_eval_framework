from setuptools import find_packages, setup

package_name = 'lidar_zones'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={'lidar_zones.zones_gen': ['config/*.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/lidar_bench.urdf']),
        ('share/' + package_name + '/launch', ['launch/robot_description.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_bench_tf_broadcaster = lidar_zones.lidar_bench_tf_broadcaster:main',
        ],
    },
)
