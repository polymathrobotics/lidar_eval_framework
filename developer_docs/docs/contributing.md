---
sidebar_position: 4
title: Contribute
---

# 🤝 Contribute

We welcome pull requests! To keep the deployment and integration pipelines clean and stable, please adhere to our shared development lifecycle rules.



# Developers: How to contribute to the framework

In terms of contributing to the core framework, there are many different ways a dev can really help out which are listed as follows:

**Legend**

- **✏️ Hands-on** — steps where *you* actually add or change code.
- **Unmarked** — background: how things work, so the hands-on parts make sense.

## 1. Contributing to `lidar_metrics_library`

The metrics library is a pure-Python plugin engine — no ROS involved. To add a metric you drop in a
file and register it. You won't touch the engine.

### First: spatial vs projective

Every metric picks a `category` — `spatial` or `projective` — and that pick decides **which cloud it
gets**. Each zone is pre-filtered both ways, so you just choose one:

- **`spatial`** → points inside the zone's 3D box.
- **`projective`** → points whose *rays* fall in the zone's angular window (already padding-trimmed).

That's the whole decision. It also sets which folder your file lives in — more below.

### How the engine works

`LidarMetricsEngine` ([engine.py](../../lidar_metrics_library/lidar_metrics/engine.py)) does three
things:

1. **Reads the registry.** [registry.yaml](../../lidar_metrics_library/lidar_metrics/registry.yaml)
   groups metrics by geometry — one key per zone type (`planar_zone_metrics`,
   `cylindrical_zone_metrics`, …). New geometry = new key, no code change.
2. **Routes each scan.** Every scan, the engine sends each metric only the zones matching its geometry
   (a planar metric never sees a cylindrical zone) and only the cloud matching its `category`. So your
   metric receives exactly the points it asked for, already bucketed per zone.
3. **Reduces at the end.** It collects every metric's result and pivots it into
   `{ zone → Metric → sub → value }` for the report. Your keys are zone-prefixed: emit `f'{zone}_foo'`
   and it lands at `report[zone][Metric][foo]`. Keys with no zone go under `__global__`.

One more thing worth knowing: **a metric is created once and reused for every scan**, so keep your
running totals in the instance.

### ✏️ Writing a new metric

**Step 1 — drop the file in the right folder.** The path is the convention:

```
zone_metrics/<geometry>_zones/<category>_metrics/<file>.py
```

e.g. a projective planar metric → `zone_metrics/planar_zones/projective_metrics/my_metric.py`. The
folder *is* how you pick your cloud (`spatial_metrics/` → box cloud, `projective_metrics/` → frustum
cloud), and it has to match the `category` you register or the import fails.

**Step 2 — subclass `MetricsBase`** ([metrics_base.py](../../lidar_metrics_library/lidar_metrics/metric_interfaces/metrics_base.py))
and fill in four methods. The rule of thumb: **all your state goes in `__init__`** so `shutdown()` can
wipe it clean for the next run.

```python
from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np

class MyMetric(MetricsBase):
    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self._sums: dict[str, float] = {}          # your accumulators live here
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self):                # once, before any scans — read config + profiles
        self._tol = self.config['lidar_metrics_parameters']['my_metric']['tolerance_m']

    def update(self, pointcloud_by_zone):   # once per scan — accumulate, don't return
        for zb in self.profiles.zone_bounds:
            pts = pointcloud_by_zone.get(zb.name)
            ...

    def compute(self):              # once at the end — return your zone-prefixed results
        return {f'{zone}_error_m': v for zone, v in self._sums.items()}

    def shutdown(self):             # once after compute — reset so the instance can be reused
        self._sums.clear()
```

Think of it as a lifecycle: `setup` (prep) → `update` (per scan) → `compute` (final answer) →
`shutdown` (clean up).

What you get to work with:

- **`self.profiles`** — just your geometry's zones. Loop `profiles.zone_bounds`; each planar bound has
  `y_min/y_max`, `z_min/z_max`, `x_surface`, `expected_depth_m`, `y_padding`/`z_padding`, and
  `profiles.lidar_position` is the sensor's xyz.
- **`self.horizontal_resolution` / `self.vertical_resolution`** — the sensor's angular resolution in
  degrees, ready by the time `setup()` runs.
- **The per-scan clouds** — a `dict[zone_name → (N, 4) xyzi array]`, already filtered per zone. Don't
  re-filter projective clouds (padding is applied upstream) — but do account for padding when scoring
  a rate, or the empty edge reads as dropout.

**Step 3 — register it** in [registry.yaml](../../lidar_metrics_library/lidar_metrics/registry.yaml)
under the right geometry key:

```yaml
    - name: MyMetric              # the class name
      description: What it measures and why.
      executable: my_metric       # the filename, minus .py
      category: projective        # spatial | projective — must match the folder
      return_type: dict[str, float]
      enabled: true               # false = keep it listed but skip it
```

`name` is the class, `executable` is the file — the engine puts them together to import your metric.

### ✏️ Tuning parameters — `config.yaml`

Any knobs your metric needs live in
[config.yaml](../../lidar_metrics_library/lidar_metrics/config.yaml) under your metric's name, and you
read them in `setup()`. Adding some is just a new block:

```yaml
lidar_metrics_parameters:
  my_metric:
    tolerance_m: 0.1
    n_bins: 5
```

### ✏️ When a parameter depends on the scene

Sometimes a knob shouldn't be a fixed number — it should follow the zones (e.g.
`spatial_dropout.cell_size_m` scaling with how far the target sits from the sensor). Two ways to do it:

- **Override plugin (cleaner).** Add a file named `<metric>__<param>__override.py` in
  [metric_params_overrides/](../../lidar_metrics_library/lidar_metrics/metric_params_overrides/),
  subclass `OverrideInterfaceBase`, and compute the value from `self.profiles` in `retrieve_param()`.
  The engine finds it by filename — no registration — and writes the result into `config.yaml` before
  the run.
  [Here's the worked example](../../lidar_metrics_library/lidar_metrics/metric_params_overrides/spatial_dropout__cell_size_m__override.py).
  Keeps the math out of your metric.
- **Just compute it in `setup()`.** Less to set up, but you're baking that logic into the metric.
  Fine for a one-off; reach for an override plugin when it's reusable or you want to test it on its own.

### ✏️ Adding metrics for a brand-new zone type

If you add a new zone geometry (covered later), the metrics side just follows the same pattern — still
no engine changes:

1. Make the folders: `zone_metrics/<new_geo>_zones/{spatial,projective}_metrics/`.
2. Add a `<new_geo>_zone_metrics:` key in `registry.yaml` and list your metrics.
3. Write them against `MetricsBase` — every metric, whatever its zone type, inherits from
   [metrics_base.py](../../lidar_metrics_library/lidar_metrics/metric_interfaces/metrics_base.py).

Match the naming (`<Geo>ZoneBounds`, `<geo>_zone_metrics`, `<geo>_zones/`) and the engine wires it all
up on its own.



## Opening a Pull Request (PR)
1. Isolate your work by creating a feature branch off of `main`: `feat/add-sensor-x` or `fix/matrix-transform`.
2. Format your commit descriptions using standard **Conventional Commits** formatting keys (e.g., `feat(core): add type-safe transform matrix`).

