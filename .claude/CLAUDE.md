# CLAUDE.md — lidar_test_bench

Guidance for Claude Code when working in the LiDAR test bench. This subtree lives inside the
`polymath_workspace` monorepo; the workspace-wide rules in the root `.claude/CLAUDE.md` (C++/Python
style, copyright headers, build commands, "never commit `src/`") still apply.

## What this is

A LiDAR evaluation test bench: it plays back (or streams) point clouds of a known static scene,
filters each scan into per-zone regions of interest, computes a battery of quality metrics per zone
against ground-truth geometry derived from TF, writes a per-case `report.yaml`, and pushes both the
metrics and a 3D visualization snapshot to Google Sheets/Drive (and optionally Notion/Grafana). A
separate Streamlit app (PolyView) reads the results back for visual analysis.

The bench evaluates one **environment** (physical scene with obstacles/zones) × one **LiDAR**
(sensor + mount pose + driver params) at a time, optionally sweeping parameters/angles into many
"cases".

## End-to-end flow

```
polysetup (CLI, standalone)
  reads environment_configs/*.yaml + lidar_configs/*.yaml
  └─ builds the bench URDF via lidar_eval_fixture, writes downstream config files
        │
        ▼
ros2 launch lidar_test_bench_bringup lidar_test_bench_launch.yaml
  brings up the nodes below
        │
raw PointCloud2 ──► lidar_baseline_node (lidar_transforms)
        │              • computes baseline profiles from TF + ROI config
        │              • hosts /get_profiles (GetProfiles) and /roi_filter (FilterCloud)
        │              • publishes filtered + projective-filtered clouds
        │              • pushes viz (expected zones + projective cloud) to /visualization
        ▼
lidar_controller (lidar_test_bench)  ◄── /start_evaluation (SetBool) starts/stops a run
        • fetches profiles via /get_profiles
        • plays bags (LidarBagPlayer), and per scan calls /roi_filter
        • feeds per-zone clouds into LidarMetricsEngine (lidar_metrics_library)
        • writes metrics_results/<env>/<lidar>/<case>/report.yaml
        • optionally syncs to Notion (DatabaseHandler)
        • calls /report_metrics when a case/run completes
        ▼
metrics_reporting_node (lidar_reporting)  ◄── /report_metrics (Trigger)
        • reads report.yaml (MetricsReader), reports to Prometheus/Grafana
        • snapshots viz blocks per case; on final push syncs metrics + viz tab to Google Sheets/Drive
        ▼
PolyView (polyview_app, Streamlit, standalone)
        reads results + viz from Google Sheets/Drive and renders 3D scene, planes, dead cells, charts
```

### Runtime service graph (who hosts what)

| Service / topic        | Type            | Host node            | Clients / notes |
|------------------------|-----------------|----------------------|-----------------|
| `/start_evaluation`    | `SetBool`       | lidar_controller     | external trigger; `data: true` starts, `false` stops |
| `/get_profiles`        | `GetProfiles`   | lidar_baseline_node  | lidar_controller |
| `/roi_filter`          | `FilterCloud`   | lidar_baseline_node  | lidar_controller; returns per-zone spatial **and** projective clouds |
| `/report_metrics`      | `Trigger`       | metrics_reporting_node| lidar_controller (snapshot per case, final push) |
| `/visualization`       | `Visualization` | metrics_reporting_node| lidar_baseline_node (client) |

`ros2 service call /start_evaluation std_srvs/srv/SetBool "{data: true}"` kicks off a run.

## Packages

ROS 2 packages (built with colcon) unless noted "standalone".

- **lidar_test_bench** — core node package. `lidar_controller` (orchestrates a run: profiles → bag
  playback → ROI filter → metrics engine → report.yaml + Notion + /report_metrics) and `lidar_node`.
  Tools in `lidar_test_bench/tools/`: `lidar_processor.py`, `bag_runner.py` (`LidarBagPlayer`).
  Params from `config/lidar_test_bench.yaml`.

- **lidar_transforms** — TF + geometry + filtering. Nodes: `lidar_baseline_node` (profiles,
  `/get_profiles`, `/roi_filter`, viz push) and `lidar_bench_tf_broadcaster`. Tools:
  `profile_builder.py` (builds `BaselineProfiles`), `zones_utilities.py` (`PlanarZoneBounds`,
  `CylindricalZoneBounds`, expected-zone + marker registries), `roi_filter.py` (spatial box +
  projective frustum filters → `FilterResult`), `roi_loader.py`, `marker_builder.py` (RViz markers),
  `profiles_serializer.py` (JSON in/out for `/get_profiles`), `transform_utils.py`.

- **lidar_metrics_library** — the metrics engine (pure Python, not a ROS node). See its own section.

- **lidar_reporting** — reporting/sinks. Nodes: `metrics_reporting_node` (Prometheus/Grafana +
  Google Sheets/Drive sync + viz tab) and `visualizer_node`. Tools: `metrics_reader.py` (loads
  report.yaml tree → `{env: {lidar: {case: data}}}`), `lidar_database_handler.py`,
  `google_services_handler.py`, `database_handler.py` (Notion). Vendors a full `polymath_core/`
  checkout under `lidar_reporting/polymath_core/` — that is upstream core, not bench code; don't edit.

- **lidar_test_bench_interfaces** — msg/srv definitions (see Interfaces below).

- **lidar_test_bench_bringup** — launch only. `launch/lidar_test_bench_launch.yaml` is the main entry
  (YAML launch); `launch/robot_description.launch.py` starts robot_state_publisher with the URDF.

- **lidar_automation_manager** — orchestrates full automated runs (driver lifecycle, angle/servo
  commands, parameter sweeps, bag recording). Node: `automation_manager`. Currently commented out in
  the main launch.

- **arduino_eth_bridge** — bridges servo/angle commands to an Arduino over TCP/Ethernet (mechanical
  LiDAR mount rotation). Node: `arduino_eth_bridge_node`. Commented out in the main launch.

- **lidar_eval_fixture** — builds the bench URDF from environment + LiDAR configs (cart geometry +
  mount + zones). Library only (no console scripts); used by polysetup.

- **lidar_configs/** — per-sensor YAMLs (AT128, JT128, robosense, innoviz, movia, robin, …): TF
  frame, cloud topic, mount pose, driver command, sweep angles, parameter-sweep definitions.

- **environment_configs/** and **config/** — per-scene definitions (zones, obstacles, world offsets,
  bag dirs/durations) and parameter overrides.

- **polysetup/** — **standalone** CLI (`cli.py`, `configure_new_run.py`, `polysetup_utils.py`).
  Run with `python3 cli.py --src-dir <ws> --lidar-file <yaml> --env-file <yaml>`. Loads env + LiDAR
  config, builds the URDF, writes downstream configs. Needs the workspace sourced (imports
  `lidar_zones`). Not a ROS package.

- **polyview_app/** — **standalone** Streamlit app (`src/app.py`, `src/visualization_handler.py`
  [plotly], `src/database_handler.py` [Google Sheets]). Run with `streamlit run src/app.py`. Reads
  results/viz from Drive and renders. Not a ROS package.

## Interfaces (lidar_test_bench_interfaces)

- `FilterCloud.srv` — req `PointCloud2 cloud`; resp `zone_names[]`, `spatial_clouds_per_zone[]`,
  `projective_clouds_per_zone[]`, `success`, `message`. **Spatial = 3D box per zone; projective =
  angular frustum per zone.** A metric's `category` decides which it receives.
- `GetProfiles.srv` — resp `profiles_json` (serialized `BaselineProfiles`), `success`, `message`.
- `Visualization.srv` — req `Visualization viz_msg`; resp `success`, `message`.
- `Visualization.msg` — `NumericalPointCloud roi_cloud`, `ExpectedZone[] expected_zones`,
  `pitch/roll/yaw`. (Only the projective cloud is pushed, once, as `roi_cloud`.)
- `ExpectedZone.msg` — `name`, `geometry` ("planar"|"cylindrical"), plus planar (`x`, `y_min/max`,
  `z_min/max`) and cylindrical (`center_x/y`, `radius`, `z_min/max`) fields.
- `NumericalPointCloud.msg` / `Point4D.msg` — `Point4D[]` of `x,y,z,intensity`.
- `Plane3D.msg` — fitted/expected plane (normal, centroid, distance, rms_error).

## lidar_metrics_library (the metrics core)

Pure-Python plugin engine. Layout under `lidar_metrics/`:

- `engine.py` — `LidarMetricsEngine`. Per scan, `run(spatial_by_zone, projective_by_zone)` routes
  each metric the dict matching its `category`. `report()` calls each plugin's `compute()` then
  `shutdown()`, pivots results into `{zone: {Metric: {sub: value}}}` (run-global keys land under
  `__global__`), and hands them to the reporter. Injects `horizontal_resolution`/`vertical_resolution`
  (degrees) before `setup()`.
- `metric_interfaces/metrics_base.py` — `MetricsBase` ABC. Lifecycle: `__init__` → `setup()` once →
  `update(pointcloud_by_zone)` per scan (accumulate, no return) → `compute()` once (reduce to dict) →
  `shutdown()` (clear accumulators so the instance is reusable). Clouds arrive pre-bucketed per zone.
- `registry.py` + `registry.yaml` — declares each metric: `name` (class), `executable` (module),
  `category` (`spatial`|`projective`), `enabled`. Import path is convention-driven:
  `zone_metrics/<geometry>_zones/<category>_metrics/<executable>.py`. **The file's directory and its
  registry `category` must agree, or it won't load.**
- `config.yaml` — per-metric parameters (e.g. `spatial_dropout.cell_size_m`).
- `reporter.py` — writes `metrics_results/<env>/<lidar>/<case>/report.yaml`.
- `zone_metrics/{planar,cylindrical}_zones/{spatial,projective}_metrics/` — the metrics.

### Profiles available to a metric (from `self.profiles`, restricted to its geometry)

- `zone_bounds[i]` (`PlanarZoneBounds`): `name`, `y_min/y_max`, `z_min/z_max`, `x_surface`,
  `expected_depth_m`, `x_min/x_max`, `y_padding/z_padding` (projective inset, per side).
- `lidar_position`: `ndarray(3,)` xyz in map frame.
- `frustrum_filter[i]`: precomputed per-zone `min/max_azimuth`, `min/max_elevation`.

### Conventions when adding/editing a metric

- Result keys are zone-prefixed (`<zone>_<sub>`); the engine strips the prefix when pivoting, so a
  metric's emitted `{zone}_foo` becomes `report[zone][Metric][foo]`. Run-global scalars (no zone
  prefix) land under `__global__`.
- Ray bearing convention (relative to `lidar_position`): `az = arctan2(dy, dx)`,
  `el = arctan2(dz, sqrt(dx²+dy²))`.
- The projective cloud is already frustum/padding-filtered upstream — don't re-filter points, but
  **do** account for padding when scoring a rate over a grid (otherwise the empty padding ring reads
  as dropout). The dropout metrics score only cells whose centers fall inside the zone bounds shrunk
  by `y_padding`/`z_padding`.
- A planar zone's surface is the plane `x = x_surface` (front of the lidar; `x_surface > lidar_x`).

## Gotchas

- **report.yaml is 3 levels deep** (`zone → Metric → sub → value`). Anything consuming it
  (Sheets flatten, viz block builders) must descend all three; 2-level walkers silently produce
  nothing. The reporting `_flatten_metrics` / `_fitted_plane_blocks` / `_dead_cell_blocks` were all
  bitten by this.
- **Registry vs file location must match** the `category`/geometry convention or import fails. Watch
  for stale duplicate metric files across `spatial_metrics/` and `projective_metrics/`.
- `metrics_results/` is ephemeral run output (gitignored) — don't rely on it persisting.
- `lidar_reporting/polymath_core/` is a vendored upstream checkout; bench code is everything else.

## Running

- Configure a run: `python3 polysetup/cli.py --src-dir <ws> --lidar-file <yaml> --env-file <yaml>` (generate URDF/configs).
- Launch: `ros2 launch lidar_test_bench_bringup lidar_test_bench_launch.yaml`.
- Start evaluation: `ros2 service call /start_evaluation std_srvs/srv/SetBool "{data: true}"`.
- View results: `streamlit run polyview_app/src/app.py`.
- Build/test from the workspace root with colcon (see root CLAUDE.md aliases: `cbp`, `cbpu`, `ctp`).
