# PolySetup

A command-line tool that configures the LiDAR test bench for a specific run. Given a **lidar config**
and an **environment config**, it generates the bench URDF and writes all the downstream node config
files (ROI regions, the automation manager's pan angles, the bringup launch parameters, …) so the
framework is ready to launch.

PolySetup is **not** a ROS node, but it imports the built `lidar_zones` package, so the ROS 2 workspace
must be sourced before you run it.

## Usage

```bash
source /lidar_test_bench/install/setup.bash            # if your shell isn't already sourced
python3 cli.py \
    --src-dir /lidar_test_bench \
    --lidar-file /lidar_test_bench/lidar_configs/robosense.yaml \
    --env-file /lidar_test_bench/environment_configs/rocinante.yaml
```

### Arguments

| Flag | Required | Description |
|------|----------|-------------|
| `--src-dir` | yes | Workspace root the packages live under. Every generated config path is built from this at runtime, so PolySetup works regardless of where it's invoked from. |
| `--lidar-file` | yes | Path to the lidar config YAML (frame, mount pose, topic, FOV, sweep params). |
| `--env-file` | yes | Path to the environment config YAML (zones, world placement, bag dirs). |
| `--backend-file` | no | Path to a backend config YAML, if your setup needs one. |

A missing or malformed config fails fast with a clear `[ERROR] …` message instead of a traceback.

## Layout

- `cli.py` — argument parsing + entry point (`main_setup`).
- `configure_new_run.py` — `ConfigureNewRun`: builds the URDF and writes every downstream config,
  with all paths rooted at `--src-dir`.
- `polysetup_utils.py` — servo pan-angle math and zone-center resolution.
