---
sidebar_position: 3
title: Developer Guide
toc_min_heading_level: 2
toc_max_heading_level: 4
---


# 1. Running the Docker container

The whole framework runs inside a ROS 2 Humble development container so you don't have to install
drivers, ROS, or Python dependencies on your host. To spin it up, from the root of the repo just run:

```bash
./run_container.sh
```

That script (`run_container.sh`) does everything for you:

- **Cleans up** any previous container named `my-lidar-bench` so ports and names are freed.
- **Builds** the image `lidar-framework:humble` from `.devcontainer/Dockerfile`, targeting the
  `ros-humble` build stage.
- **Launches** the container with `--net=host` and `--ipc=host` (so ROS 2 discovery and shared-memory
  transport work against sensors and other nodes on your machine), and **mounts the entire repo**
  into the container at `/workspace/<repo-dir-name>`, which is also set as the working directory.

Because the repo is bind-mounted (not copied), any edit you make on the host shows up instantly inside
the container and vice-versa. When you exit the shell the container is removed (`--rm`), but the image
is cached, so subsequent launches are fast.

> If you use VS Code, the same `.devcontainer/Dockerfile` backs the "Reopen in Container" workflow, so
> you get an identical environment either way.

Once you're inside the container, build the workspace with `colcon` (see the aliases in the root
`CLAUDE.md`) before launching anything.

# 2. Authenticating your database

To push your lidar data, metrics, and visualization snapshots to a database, you must authenticate
**before** you launch the main framework — otherwise the reporting node will come up unauthenticated
and silently skip the sync.

The default backend (`GoogleServicesHandler`, in
[lidar_eval_backends/.../google_services_handler.py](../../lidar_eval_backends/lidar_eval_backends/database_backends/google/google_services_handler.py))
needs a Google service-account key plus the Drive folder to write into. Out of the box it reads them
from a plain `KEY=VALUE` env file, so set yours up once:

1. Look at [lidar_eval_backends/auth.env.example](../../lidar_eval_backends/auth.env.example) — it
   lists every field the backend expects: the service-account JSON fields verbatim, plus your Drive
   `root_folder_id`.
2. Grab your own Google Cloud service-account details from **1Password** and fill them into your own
   copy of that file. Escape the newlines in `private_key` as `\n`.
3. Save your copy as `lidar_eval_backends/auth.env` and keep it there. It's gitignored and never
   installed into `install/`, so your credentials stay out of the repo and off the overlay.

That's the whole setup — the `Env` provider is enabled by default, so your next launch picks the file
up with no further changes. To keep it somewhere else, set `$AUTH_ENV_FILE` or edit `env_file` on the
`Env` row in
[google/auth_registry.yaml](../../lidar_eval_backends/lidar_eval_backends/database_backends/google/auth_registry.yaml).
A wrong `$AUTH_ENV_FILE` fails loudly rather than quietly falling back to the configured path.

:::note If you're at Polymath, enable 1Password instead

Don't keep an `auth.env` on disk at all. In
[google/auth_registry.yaml](../../lidar_eval_backends/lidar_eval_backends/database_backends/google/auth_registry.yaml)
flip the two providers:

```yaml
authentication_registry:
  - authentication_backend: OnePassword
    enabled: true      # was false
  - authentication_backend: Env
    enabled: false     # was true
```

Then **run `op signin` every time before you launch the framework**. With a live session the
`OnePassword` provider pulls the service-account key and `root_folder_id` straight out of Core for you
— the shared **"Lidar Evaluation Results Database"** item in the `Employee` vault — so there are no
credentials on your disk. If it finds no live `OP_SESSION_*` token it runs `op signin` itself and
prompts you to unlock, which needs a TTY; sign in up front when you're launching from a script.
:::

## Using your own store / your own backend

You are not tied to 1Password or Google. Credentials and storage are separate plugin points, so pick
the one that matches what you're swapping:

- **Same database, different secret store** — write a credential provider. Add a class implementing
  [`AuthInterface`](../../lidar_eval_backends/lidar_eval_backends/authentication_interface.py) under
  `lidar_eval_backends/lidar_eval_backends/database_backends/google/auth/`, returning the same blob
  shape the Google backend expects, and enable it in that backend's `auth_registry.yaml` (disabling
  the others). Its `config:` block is handed to your provider on construction, so declare any paths
  or item references there rather than deriving them in code. The storage handler never changes.
- **A different database entirely** — write a storage backend. Add a class implementing
  [`DatabaseInterface`](../../lidar_eval_backends/lidar_eval_backends/database_interface.py)
  (`authenticate()`, `load_credentials()`, `sync()`, and the read half PolyView uses) under
  `lidar_eval_backends/lidar_eval_backends/database_backends/`, then register it in
  [database_registry.yaml](../../lidar_eval_backends/lidar_eval_backends/database_registry.yaml)
  with its `class`, `executable` (module path), and `enabled: true`.

Both registries load the **first enabled** row, so make sure only the one you want is enabled.

---

Now, to make the framework work for any lidar in any environment, you (the developer) write **two YAML
files**: one describing the **lidar** (topic, frame, mount pose, driver, sweepable parameters) and one
describing the **environment** (the physical scene and its zones). These live in `lidar_configs/` and
`environment_configs/` respectively.

# 3. Building a lidar config file

A lidar config (e.g. [lidar_configs/AT128P.yaml](../../lidar_configs/AT128P.yaml),
[lidar_configs/robosense.yaml](../../lidar_configs/robosense.yaml)) tells the bench everything it needs
to know about a sensor: how to identify it, where it's mounted, how to launch its driver, and which
driver parameters to sweep. Every field lives under a top-level `lidar:` key.

### Identity, pose, and geometry

```yaml
lidar:
  # ── Identity ──
  frame:               hesai_lidar      # TF frame / URDF link name for this sensor
  folder:              AT128            # results folder + database identifier
  point_cloud_topic:   /lidar_points    # the PointCloud2 topic the driver publishes

  # ── Mount pose (the lidar_joint in the generated URDF) ──
  joint_xyz_m:         [0.0, 0.0, 0.05] # sensor offset from the motor axis, meters
  joint_rpy_rad:       [0.0, 0.0, 0.0]  # sensor orientation, radians (roll, pitch, yaw)

  # ── Bench geometry ──
  motor_to_lidar_height_m: 0.1524       # ground-to-cart-base vertical distance (map_to_cart z)
```

These drive the URDF/TF, so the bench knows where the sensor sits relative to the scene and can derive
ground-truth zone geometry.

### Driver

```yaml
  ros2_driver:        hesai_ros_driver
  driver_command:     'ros2 launch hesai_ros_driver start.py'   # how the automation manager starts it
  driver_config_file: '/workspaces/.../HesaiLidar_ROS_2.0/config/config.yaml'  # the driver's own config
  lidar_gui:          ''    # leave empty to disable the GUI prompt (input() needs a TTY)
```

`driver_config_file` is the sensor driver's **own** YAML — the bench edits keys inside it when sweeping
parameters (see below).

### Sensor specs

Descriptive fields used for reporting and (eventually) geometry-based angle computation:

```yaml
  horizontal_fov_deg:        120.0
  vertical_fov_deg:          25.4
  horizontal_resolution_deg: 0.2    # injected into the metrics engine as horizontal_resolution
  vertical_resolution_deg:   0.8    # injected as vertical_resolution
  cost:                      1800.0
```

### Sweeps — the key idea

This is where a single lidar file expands into **many test cases**. Two things drive a sweep — the
**servo angles** the mount sweeps through, and the **driver parameters** you vary.

#### Angles are computed for you — you don't write them

You do **not** hand-write a list of servo angles. Instead, you just provide the sensor's field of view
(`horizontal_fov_deg` and `vertical_fov_deg` from the Sensor specs above) and the environment's zone
layout, and polysetup does the geometry for you.

When you configure a run, polysetup's `compute_panning_angles`
([polysetup/polysetup_utils.py](../../polysetup/polysetup/polysetup_utils.py)) works in three steps:

1. **Resolve every zone.** It walks the environment file's `world_placement` + `zone_joints` to find
   each zone's center, then uses each zone's `length` to expand that center out to its **edges**.
2. **Combine into one edge-to-edge boundary.** It takes *all* the zones together and computes the
   outer bounding region that encloses them — the leftmost edge across all zones, the rightmost edge
   across all zones, and the nearest face (plus a `breathing_room_m` buffer on each side). This single
   edge-to-edge region is what the sweep has to keep in view.
3. **Solve pan angles from that boundary.** It then computes the pan angles at which the lidar's FOV
   just contains that combined region, emitting **5 angles** —
   `[left_extreme, left_mid, 0.0, right_mid, right_extreme]` — that span the coverage so the zones are
   seen from one edge of the field of view to the other.

Those computed angles are written straight into `automation_manager.yaml`
([polysetup/configure_new_run.py](../../polysetup/polysetup/configure_new_run.py)); the automation manager reads
them from there at launch. So the only inputs you maintain are the FOV numbers and the zone geometry.

> **Current scope:** angle computation uses the **horizontal** FOV to compute the pan (left/right)
> angles from the zones' edge-to-edge boundary. `vertical_fov_deg` is still required — it's carried
> into the reporting/resolution config — but it does not yet drive tilt angles. (You may still see a
> leftover `angles:` field in some lidar config files; it's vestigial and is overwritten by the
> computed values.)

#### Parameters — what you actually sweep

**`parameter_names`** is the list of driver parameters you want to vary. Each name **must** have a
matching parameter block further down.

```yaml
parameter_names: [min_distance, max_distance, dense_points, start_angle, end_angle]
```

Each parameter block declares its type, a default, an **array of values to try**, and the dotted
`path` into the driver's own config file where that value gets written:

```yaml
  min_distance:
    type:    double
    default: 0.2
    values:  [0.2, 1.0, 5.0]                 # ← each value becomes a case
    path:    lidar.0.driver.min_distance     # where in driver_config_file to write it
    description: Minimum range threshold in meters. Returns closer than this are filtered out.
```

| Field         | Meaning                                                                 |
|---------------|-------------------------------------------------------------------------|
| `type`        | `bool` / `int` / `double` — how the value is written into the driver.   |
| `default`     | Value used when this parameter isn't the one being swept.               |
| `values`      | The list of options to iterate over; each produces a distinct case.     |
| `path`        | Dotted key path into `driver_config_file` that the bench overwrites.    |
| `description` | Optional human-readable note (shown in reporting).                      |

At run time the automation manager walks `angles` × the swept `parameter_names`, writes each value to
the driver config at its `path`, records a bag for `bag_recording_duration` seconds (set in the
environment file), and produces one `report.yaml` per case.

# 4. Building an environment config file

An environment config (e.g.
[environment_configs/coop_plus_maiku_hangout.yaml](../../environment_configs/coop_plus_maiku_hangout.yaml),
[environment_configs/pole_test.yaml](../../environment_configs/pole_test.yaml)) describes the **physical
scene**: the target zones the bench scores against ground truth, how those zones are positioned
relative to each other, and where the whole scene sits in the world. All dimensions are in **meters**.

### Top-level fields

```yaml
name: coop_plus_maiku_hangout

bag_recorder_directory: '/workspaces/polymath_workspace/COOP_HANGOUT_BAG'  # where recordings go
bag_recording_duration: 20                                                 # seconds per case

zones:
    - "whiteboard"       # ordered list of the zones present in this scene
    - "blue_suitcase"
```

### Zone properties

Each zone named in `zones` needs an entry in `zone_properties`. There are two `type`s:

**Planar zones** (a flat rectangular target like a wall or whiteboard):

```yaml
    whiteboard:
        frame:     "whiteboard"      # TF frame / URDF link for this zone
        type:      "planar"
        color:     [255, 255, 255]   # RGB, used for visualization
        height:    1.1684            # zone height (z extent)
        length:    2.4003            # zone width  (y extent)
        z_offset:  0.6985            # height of the zone's base off the ground
        y_padding: 0.1016            # inset per side, ignored when scoring (see gotchas)
        z_padding: 0.1016
```

**Cylindrical zones** (a round target like a pole or half-cylinder):

```yaml
    white_half_cylinder:
        frame:          "white_half_cylinder"
        type:           "cylindrical"
        color:          [255, 255, 255]
        height:         0.127
        radius:         0.04826
        position:       "forward"     # which side of the cylinder faces the sensor
        z_offset:       0.9398
        radius_padding: 0.01
        height_padding: 0.02
```

The `*_padding` values define an inset ring the metrics engine excludes when scoring rates (so the
empty edge of a filtered cloud doesn't read as dropout). They're optional — omit them (as in
[environment_configs/rocinante.yaml](../../environment_configs/rocinante.yaml)) for no inset.

### Connecting zones — `zone_joints`

When a scene has more than one zone, you position them **relative to each other** with `zone_joints`
rather than giving every zone absolute world coordinates. You anchor one zone and attach the others to
its left/right with x/y offsets:

```yaml
zone_joints:
  whiteboard:                 # the anchor/parent zone
    left_zone:                # attach another zone to its left (use right_zone for the other side)
      zone: blue_suitcase     # the child zone being placed
      x_offset: -0.1524       # how far forward/back of the parent it sits
      y_offset: 0.127         # lateral offset
```

### Placing the scene — `world_placement`

Finally, anchor the whole scene in the world by picking one `child_zone` and giving its offset from
the world origin:

```yaml
world_placement:
    child_zone: "whiteboard"   # the zone the world is measured to
    x_offset: 3.81             # distance of that zone from the origin, meters
    y_offset: 0.0
```

Everything else in the scene is positioned relative to that anchor via `zone_joints`, so moving the
whole scene is a one-line change. (The inline comments note this placement will eventually move into
polysetup.)


# 5. how do you build a default file for your platform

TODO - Talk about this soon

# 6. Configuring a run with PolySetup

Once you've written a lidar config and an environment config (sections 3 and 4), **PolySetup** is the
command-line tool that ties them together: you point it at the two YAML files, and it generates the
bench URDF and writes all the downstream config files (ROI regions, the automation manager's computed
angles, the bringup launch parameters, etc.) so the framework is ready to launch.

PolySetup is a standalone CLI — it is **not** a ROS node, so you run it directly with Python. It needs
the workspace sourced (it imports the built `lidar_zones` package; the devcontainer sources it for you):

```bash
source install/setup.bash            # if your shell isn't already sourced
python3 polysetup/cli.py \
    --src-dir /lidar_test_bench \
    --lidar-file lidar_configs/AT128P.yaml \
    --env-file environment_configs/rocinante.yaml
```

Arguments:

- `--src-dir` — the workspace root the packages live under. Every generated config path is built from
  this at runtime, so PolySetup works regardless of where it's invoked from.
- `--lidar-file` — path to your lidar YAML (section 3).
- `--env-file` — path to your environment YAML (section 4).
- `--backend-file` *(optional)* — a backend config file, if your setup needs one.

It loads the two YAMLs, builds the URDF, and writes every downstream config. A missing or malformed
file fails fast with a clear `[ERROR] …` message instead of a traceback.

After it finishes you can launch the bench:

```bash
ros2 launch lidar_test_bench_bringup lidar_test_bench_launch.yaml
```

# 7. Viewing results in PolyView (and publishing a public link)

**PolyView** is a standalone Streamlit app that reads the evaluation results and visualization
snapshots back from Google Drive/Sheets and renders the 3D scene, fitted planes, dead cells, and
charts. Like PolySetup it is not a ROS node.

## Running it locally from the terminal

From the repo root:

```bash
cd polyview_app
pip install -r requirements.txt          # first time only
streamlit run src/app.py
```

Streamlit prints a local URL (default `http://localhost:8501`) and opens it in your browser.

PolyView reads from the same Google service-account database that the reporter writes to, so it needs
credentials. Locally these live in `polyview_app/.streamlit/secrets.toml`: a top-level `root_folder_id`
(the Drive folder holding the results tree) and a `[google_sheets]` section holding the
service-account JSON fields. Populate that file with your own service account before running.

## Publishing a public link via Streamlit Community Cloud

To make the dashboard a shareable link that anyone in your organization can open (rather than only
running on your machine), deploy it on **Streamlit Community Cloud**:

1. **Sign up / sign in** at [share.streamlit.io](https://share.streamlit.io) and choose
   **Continue with GitHub**, authorizing Streamlit to access your GitHub account.
2. **Fork the PolyView repo** into your own GitHub account (Streamlit Community Cloud deploys from a
   GitHub repo you own, so it needs to live under your account). Push the `polyview_app` code there if
   it isn't already on GitHub.
3. **Create the app.** In Streamlit Cloud click **New app**, select your forked repo, pick the branch,
   and set the **main file path** to the app's full path from the repo root —
   `polyview_app/src/app.py`. There is no "working directory" setting, so the path has to be complete.
4. **Add your secrets.** Cloud has no `op` CLI and no `auth.env`, so PolyView falls back to
   `st.secrets['database_credentials']` — and if that's missing you get
   `Could not authenticate to the results database` on startup. In the app's **Advanced settings →
   Secrets** (or **Manage app → Settings → Secrets** after deploying), add a
   **`[database_credentials]`** table holding your Drive `root_folder_id` plus the service-account
   JSON fields verbatim — the same set of keys as
   [auth.env.example](../../lidar_eval_backends/auth.env.example):

   ```toml
   [database_credentials]
   root_folder_id = "YOUR_DRIVE_ROOT_FOLDER_ID"
   type = "service_account"
   project_id = "your-project-id"
   private_key_id = "xxxxxxxxxxxxxxxx"
   private_key = '''
   -----BEGIN PRIVATE KEY-----
   MIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQC...
   ...every line of the key, verbatim...
   -----END PRIVATE KEY-----
   '''
   client_email = "your-sa@your-project.iam.gserviceaccount.com"
   client_id = "xxxxxxxxxxxxxxxxxxxx"
   auth_uri = "https://accounts.google.com/o/oauth2/auth"
   token_uri = "https://oauth2.googleapis.com/token"
   auth_provider_x509_cert_url = "https://www.googleapis.com/oauth2/v1/certs"
   client_x509_cert_url = "https://www.googleapis.com/robot/v1/metadata/x509/your-sa%40your-project.iam.gserviceaccount.com"
   ```

   The table name matters — it must be `database_credentials`. Everything except `root_folder_id` is
   handed straight to Google's `from_service_account_info()`, so the key names have to match the
   service-account JSON exactly.

   **Use `'''` for `private_key`, not `"…\n…"`.** Both are valid TOML — a basic `"…"` string does turn
   `\n` into the real newlines Google needs — but that puts the whole key on one ~1750-character line
   in a web textarea, where a truncated paste or a stray `\\n` is easy and the failure mode is
   miserable to read: a `ValueError` from `load_pem_private_key` deep inside `google.auth`, with the
   message redacted by Streamlit. `'''` is a *literal* multi-line string — no escape processing at all
   — so the PEM's real line breaks go in verbatim and there are no backslashes to get wrong.

   If you do hit that `ValueError`, the key material is rarely the problem; check what's actually
   stored. A leftover `MII…` placeholder produces exactly this error, because `…` isn't valid base64.

   Never commit `secrets.toml` to the repo; Streamlit stores these securely instead.
5. **Deploy.** Streamlit builds the app (installing `requirements.txt`) and gives you a public
   `*.streamlit.app` URL. Share that link — anyone in your organization can open it to view the lidar
   evaluation data, no local setup required.

:::warning `requirements.txt` has to sit next to `app.py`

Streamlit Community Cloud looks for a dependency file in exactly two places: the **repository root**
or the **same directory as the entrypoint file**. Intermediate directories are ignored silently — no
warning, no build error. That's why the file lives at `polyview_app/src/requirements.txt` rather than
`polyview_app/requirements.txt`; moving it up a level makes the deploy fail at the first
non-Streamlit import (`ModuleNotFoundError: yaml`) with nothing in the build log to explain it.

The same rule bites `.streamlit/config.toml`: the theme in `polyview_app/.streamlit/` applies when
you run locally from that directory, but Cloud resolves it against the repo root, so the deployed app
falls back to the default theme. Set the theme under **Advanced settings** instead, or duplicate the
file at the repo root.
:::

> Because the app is public, make sure the service account you point it at only has access to the
> Drive data you actually intend to share.
