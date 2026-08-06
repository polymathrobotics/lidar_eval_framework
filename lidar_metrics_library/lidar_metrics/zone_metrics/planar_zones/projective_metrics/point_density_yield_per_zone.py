import math

import numpy as np

from lidar_metrics.metric_interfaces.metrics_base import MetricsBase


class PointDensityYieldPerZone(MetricsBase):
    """Per-zone point density expressed as actual / expected, averaged across scans.

    For each zone the expected number of returns per scan is derived from
    geometry and the lidar's angular resolution:

        expected = (w * h / (D^2 * dθ_h * dθ_v)) * cos(α)

    where w/h are the zone's Y/Z extents, D is the 3D distance from the lidar
    to the zone center, dθ_{h,v} are the lidar's horizontal/vertical angular
    resolutions in radians, and cos α = |dx|/D is the incidence-angle factor
    for axis-aligned planar zones whose surface normal points back at the
    lidar along −X.

    Per scan we record density_pct = 100 * actual / expected for each zone.
    compute() averages those per-scan percentages per zone — so a value of
    100% means the sensor is hitting that zone as densely as geometry predicts,
    <100% means it's dropping returns, >100% means it's over-sampling (e.g.
    multi-return mode).
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        # Profile-derived constants pre-computed in setup() so update() is cheap.
        # Angular resolutions are injected by the engine as attributes after
        # construction (see LidarMetricsEngine.create_plugins), so they're not
        # part of this constructor's signature.
        self._zone_expected_points: dict[str, float] = {}
        self._zone_density_history: dict[str, list[float]] = {}
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        if self.profiles is None:
            return

        lidar_pos = self.profiles.lidar_position
        h_rad = math.radians(self.horizontal_resolution)
        v_rad = math.radians(self.vertical_resolution)
        solid_angle_per_beam = h_rad * v_rad

        for zb in self.profiles.zone_bounds:
            # Match the projective frustum filter: it shrinks the zone inward by
            # y_padding/z_padding on each side. Expected has to use the same
            # shrunken footprint to be apples-to-apples with what `actual` sees.
            w = max(0.0, float(zb.y_max - zb.y_min) - 2.0 * float(zb.y_padding))
            h = max(0.0, float(zb.z_max - zb.z_min) - 2.0 * float(zb.z_padding))
            y_c = (zb.y_min + zb.y_max) / 2.0
            z_c = (zb.z_min + zb.z_max) / 2.0
            dx = float(zb.x_surface - lidar_pos[0])
            dy = float(y_c - lidar_pos[1])
            dz = float(z_c - lidar_pos[2])
            d_sq = dx * dx + dy * dy + dz * dz

            if 0.0 == d_sq or 0.0 == solid_angle_per_beam or 0.0 == w or 0.0 == h:
                self._zone_expected_points[zb.name] = 0.0
            else:
                d = math.sqrt(d_sq)
                cos_alpha = abs(dx) / d
                self._zone_expected_points[zb.name] = (w * h / (d_sq * solid_angle_per_beam)) * cos_alpha

            self._zone_density_history[zb.name] = []

    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return
        self.pointcloud_by_zone = pointcloud_by_zone

        for zb in self.profiles.zone_bounds:
            expected = self._zone_expected_points.get(zb.name, 0.0)
            if 0.0 >= expected:
                continue
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                actual = 0
            else:
                actual = int(np.isfinite(zone_pts[:, :3]).all(axis=1).sum())
            self._zone_density_history[zb.name].append(100.0 * actual / expected)

    def compute(self) -> dict[str, float]:
        if self.profiles is None:
            return {}
        result = {}
        for zb in self.profiles.zone_bounds:
            history = self._zone_density_history.get(zb.name, [])
            result[f'{zb.name}_density_pct'] = float(np.mean(history)) if history else 0.0
            result[f'{zb.name}_expected_points'] = float(self._zone_expected_points.get(zb.name, 0.0))
        return result

    def shutdown(self) -> None:
        for history in self._zone_density_history.values():
            history.clear()
