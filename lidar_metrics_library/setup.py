from setuptools import find_packages, setup

package_name = 'lidar_metrics_library'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    package_data={
        'lidar_metrics': ['*.yaml'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aarush Jain',
    maintainer_email='aarush@polymathrobotics.com',
    description='Lidar metrics evaluation library for comparing sensor performance on the lidar test bench.',
    license='Apache-2.0',
    # colcon picks its pytest runner only for packages that declare a 'test' extra on pytest.
    # Without this it falls back to the setuptools/unittest runner, which collects nothing here
    # and exits 5 ("NO TESTS RAN"), failing the package.
    extras_require={
        'test': [
            'pytest',
        ],
    },
)
