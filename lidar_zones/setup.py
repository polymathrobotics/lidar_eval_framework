from setuptools import find_packages, setup

package_name = 'lidar_zones'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'lidar_zones.zones_gen': ['config/*.yaml'],
        # The ZoneEngine loads this next to zone_engine.py at runtime, so it must
        # be installed inside the package (not just under share/).
        'lidar_zones.zones_api': ['zones_types_registry.yaml'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/roi.yaml', 'config/zones_orchestrator.yaml']),
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
            'zones_orchestrator_node = lidar_zones.zones_orchestrator_node:main',
        ],
    },
)
