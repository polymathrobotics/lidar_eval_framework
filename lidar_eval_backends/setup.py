from pathlib import Path

from setuptools import find_packages, setup

package_name = 'lidar_eval_backends'
_HERE = Path(__file__).parent
_BACKENDS_DIR = _HERE / package_name / 'database_backends'


def _read_reqs(req_file: Path) -> list:
    """Read a requirements file into a list of pins, skipping blank lines and comments."""
    return [ln.strip() for ln in req_file.read_text().splitlines()
            if ln.strip() and not ln.strip().startswith('#')]


# Each database_backends/<name>/requirements.txt becomes an install extra named <name>, plus an
# `all` extra that unions them. Adding a backend = drop in its folder + requirements.txt; the
# extras (and `all`) pick it up automatically — nothing else in setup.py to edit.
#
# These are read at BUILD time into STATIC extras. Which backend actually RUNS is a separate,
# runtime choice (database_registry.yaml), independent of which extras were installed.
_extras: dict[str, list] = {}
if _BACKENDS_DIR.exists():
    for backend_dir in sorted(p for p in _BACKENDS_DIR.iterdir() if p.is_dir()):
        req = backend_dir / 'requirements.txt'
        if req.exists():
            _extras[backend_dir.name] = _read_reqs(req)
_extras['all'] = sorted({dep for deps in _extras.values() for dep in deps})
_extras['test'] = ['pytest']

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    include_package_data=True,
    # Registry YAML lives at the package root; each backend's requirements.txt lives in its
    # folder. Both must ship as package data (registry is read via importlib.resources).
    package_data={
        'lidar_eval_backends': ['*.yaml'],
        'lidar_eval_backends.database_backends.google': ['*.yaml'],
        '': ['requirements.txt'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],   # core: only what the registry loader needs
    zip_safe=True,
    maintainer='Aarush Jain',
    maintainer_email='aarush@polymathrobotics.com',
    description='Pluggable database (storage) backends for the LiDAR '
                'evaluation framework — shared by the reporter and PolyView.',
    license='Apache-2.0',
    extras_require=_extras,
    entry_points={
        'console_scripts': [
        ],
    },
)
