# Copyright (c) 2026-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

import argparse
import sys
import textwrap
from pathlib import Path

import yaml

try:
    from polysetup.configure_new_run import ConfigureNewRun
except ModuleNotFoundError as exc:
    if exc.name in ('lidar_zones', 'lidar_zones.zones_api', 'lidar_zones.zones_gen'):
        sys.exit(
            "[ERROR] ROS workspace not sourced — polysetup needs the built 'lidar_zones' "
            "package on PYTHONPATH.\n        Run:  source <workspace>/install/setup.bash   "
            "(e.g. source /lidar_test_bench/install/setup.bash)\n        then retry."
        )
    raise


def _load_yaml(path: Path, label: str) -> dict:
    """Load and parse a YAML config file, exiting with a clear message if it's missing,
    unreadable, malformed, or empty."""
    try:
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"[ERROR] {label} '{path}' does not exist", file=sys.stderr)
        sys.exit(1)
    except yaml.YAMLError as exc:
        print(f"[ERROR] {label} '{path}' is not valid YAML: {exc}", file=sys.stderr)
        sys.exit(1)
    except OSError as exc:
        print(f"[ERROR] could not read {label} '{path}': {exc}", file=sys.stderr)
        sys.exit(1)
    if data is None:
        print(f"[ERROR] {label} '{path}' is empty", file=sys.stderr)
        sys.exit(1)
    return data


def main_setup() -> None:
    """Parse command-line arguments to run polysetup and configure the framework for user specific application

    @return  None.
    @throws SystemExit  If --src-dir is not a directory or a spec cannot be resolved.
    """

    parser = argparse.ArgumentParser(
        description=textwrap.dedent("""\
            Polysetup - A tool used for configuring the framework for user specific application

            Takes in 4 arguments: 
            
            src-dir : Workspace source directory the repositories live under (default: current directory).
            lidar-file : Path to the lidar configuration file (e.g. hesai.yaml) that defines lidar identity, mount pose, and topic.
            env-file : Path to the environment configuration file (e.g. rocinante.yaml)
            backend-file [optional] : If our specific backend application requires a backend configuration file, this argument can be used to specify the path to that file. If not provided, the default backend configuration will be used.
            
            """),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        '--src-dir',
        type=Path,
        default=Path.cwd(),
        help='Workspace source directory the repositories live under (default: current directory).',
    )

    parser.add_argument(
        '--lidar-file',
        type=Path,
        help='Path to the lidar configuration file (e.g. hesai.yaml) that defines lidar identity, mount pose, and topic.',
    )

    parser.add_argument(
        '--env-file',
        type=Path,
        help='Path to the environment configuration file (e.g. rocinante.yaml)',
    )

    parser.add_argument(
        '--backend-file',
        type=Path,
        default=None,
        help='Path to the backend configuration file (e.g. backend.yaml) that defines backend specific configurations. If not provided, the default backend configuration will be used.',
    )

    args = parser.parse_args()

    src_dir = args.src_dir.resolve()
    if not src_dir.is_dir():
        print(f"[ERROR] --src-dir '{src_dir}' is not a directory", file=sys.stderr)
        sys.exit(1)

    if args.lidar_file is None:
        print("[ERROR] --lidar-file is required", file=sys.stderr)
        sys.exit(1)
    if args.env_file is None:
        print("[ERROR] --env-file is required", file=sys.stderr)
        sys.exit(1)

    # Load each config up front so a missing/malformed file fails clearly here, not deep in the run.
    lidar_config = _load_yaml(args.lidar_file, '--lidar-file')
    env_config = _load_yaml(args.env_file, '--env-file')
    backend_config = _load_yaml(args.backend_file, '--backend-file') if args.backend_file else None

    # Configure the run: builds the URDF + all downstream node configs, with every path derived
    # from src_dir at runtime (no hardcoded config paths). Expected configuration errors — e.g.
    # the zone span exceeding the lidar FOV — surface as a clean message, not a traceback.
    try:
        ConfigureNewRun(
            lidar_config=lidar_config,
            environment_config=env_config,
            src_dir=src_dir,
        ).configure()
    except ValueError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)



def bag_recording_setup() -> None:
    """Parse command-line arguments to run polysetup and configure the framework for bag recording

    @return  None.
    @throws SystemExit  If given flag is not a valid boolean or a spec cannot be resolved.
    """

    parser = argparse.ArgumentParser(
        description=textwrap.dedent("""\
            Polysetup - A helper tool used to enable/disable the bag recording feature in the framework
            
            Takes in 1 argument:

            bag-recording-status: A boolean value (True/False) to enable or disable the bag recording feature in the framework.
            """),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        '--bag-recording-status',
        type=lambda v: v.strip().lower() in ('true', '1', 'yes', 'on'),
        default=False,
        help='Enable or disable the bag recording feature in the framework (true/false).',
    )

    parser.add_argument(
        '--src-dir',
        type=Path,
        default=Path.cwd(),
        help='Workspace source directory the repositories live under (default: current directory).',
    )

    args = parser.parse_args()
    src_dir = args.src_dir.resolve()
    if not src_dir.is_dir():
        print(f"[ERROR] --src-dir '{src_dir}' is not a directory", file=sys.stderr)
        sys.exit(1)

    # An env var set here would die with this process, so persist it in .env — just auto-loads that
    # file into every recipe, so the next `just setup-ws` picks the choice up. write_text creates
    # .env when it isn't there yet and overwrites it when it is.
    env_path = src_dir / '.env'
    created = not env_path.exists()
    value = str(args.bag_recording_status).lower()
    env_path.write_text(f'BAG_RECORDING={value}\n')
    print(f"  .env  BAG_RECORDING={value}{' (created)' if created else ''}")


def angle_detection_setup() -> None:
    """Parse command-line arguments to enable/disable the angle sweep in the framework

    @return  None.
    @throws SystemExit  If given flag is not a valid boolean or a spec cannot be resolved.
    """

    parser = argparse.ArgumentParser(
        description=textwrap.dedent("""\
            Polysetup - A helper tool used to enable/disable angle detection in the framework

            When enabled, the automation manager sweeps the mount through every angle
            computed for the lidar's FOV, recording a case per angle. When disabled
            (the default) the run is just the base case plus the parameter sweeps.

            Takes in 1 argument:

            angle-detection-status: A boolean value (True/False) to enable or disable the angle sweep.
            """),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        '--angle-detection-status',
        type=lambda v: v.strip().lower() in ('true', '1', 'yes', 'on'),
        default=False,
        help='Enable or disable the angle sweep in the framework (true/false).',
    )

    parser.add_argument(
        '--src-dir',
        type=Path,
        default=Path.cwd(),
        help='Workspace source directory the repositories live under (default: current directory).',
    )

    args = parser.parse_args()
    src_dir = args.src_dir.resolve()
    if not src_dir.is_dir():
        print(f"[ERROR] --src-dir '{src_dir}' is not a directory", file=sys.stderr)
        sys.exit(1)

    try:
        ConfigureNewRun.set_angle_detection(src_dir, args.angle_detection_status)
    except ValueError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        sys.exit(1)



if __name__ == '__main__':
    main_setup()

