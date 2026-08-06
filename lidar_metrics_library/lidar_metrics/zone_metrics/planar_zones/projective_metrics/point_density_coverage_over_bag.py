import math

import numpy as np

from lidar_metrics.metric_interfaces.metrics_base import MetricsBase


class PointDensityCoverageOverBag(MetricsBase):
    """Per-zone coverage of the lidar's advertised angular grid, accumulated
    over every scan in the bag.

    Counts UNIQUE rays (binned at the lidar's published Δθ_h × Δθ_v) that
    landed in each zone across all scans in the test run, then compares to the
    geometric maximum:

        expected = (w * h / (D² * Δθ_h * Δθ_v)) * cos(α)

    where w/h are the projective-padded zone extents, D is the 3D distance to
    the zone center, Δθ_{h,v} are the lidar's angular resolutions (radians),
    and cos α = |dx|/D for an axis-aligned planar zone facing −X.

    Difference vs. PointDensityYieldPerZone:
      - PointDensityYieldPerZone reports actual / expected averaged across
        scans (per-frame question: "is each scan delivering its quota?").
      - This metric deduplicates rays across the bag (per-bag question: "did
        the lidar ever fill the advertised grid?"). Tolerates interlaced
        scanners that need N frames to complete a full pattern.

    Limits:
      - Yields > 100% are not possible — each (az_bin, el_bin) is counted at
        most once. Multi-return sensors don't show up as over-delivering here.
      - Non-uniform / rosette scanners plateau at whatever fraction of their
        advertised grid they ever fill (which is the honest answer for them).
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self._zone_expected_points: dict[str, float] = {}
        # Per zone: set of unique (az_bin, el_bin) tuples observed over the bag.
        # Packed as int64 = (az_bin << 32) | (el_bin & 0xFFFFFFFF) for fast set ops.
        self._zone_unique_rays: dict[str, set] = {}
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        if self.profiles is None:
            return

        lidar_pos = self.profiles.lidar_position
        h_rad = math.radians(self.horizontal_resolution)
        v_rad = math.radians(self.vertical_resolution)
        solid_angle_per_beam = h_rad * v_rad

        for zb in self.profiles.zone_bounds:
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

            self._zone_unique_rays[zb.name] = set()

    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return
        self.pointcloud_by_zone = pointcloud_by_zone

        h_rad = math.radians(self.horizontal_resolution)
        v_rad = math.radians(self.vertical_resolution)
        if 0.0 >= h_rad or 0.0 >= v_rad:
            return

        lidar_pos = self.profiles.lidar_position

        for zb in self.profiles.zone_bounds:
            ray_set = self._zone_unique_rays.get(zb.name)
            if ray_set is None:
                continue
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or 0 == len(zone_pts):
                continue
            finite = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
            if 0 == len(finite):
                continue

            dx = finite[:, 0] - lidar_pos[0]
            dy = finite[:, 1] - lidar_pos[1]
            dz = finite[:, 2] - lidar_pos[2]
            az = np.arctan2(dy, dx)
            el = np.arctan2(dz, np.sqrt(dx * dx + dy * dy))
            az_bin = np.floor(az / h_rad).astype(np.int64)
            el_bin = np.floor(el / v_rad).astype(np.int64)
            combined = (az_bin << 32) | (el_bin & 0xFFFFFFFF)
            for c in np.unique(combined):
                ray_set.add(int(c))

    def compute(self) -> dict[str, float]:
        if self.profiles is None:
            return {}
        result: dict[str, float] = {}
        for zb in self.profiles.zone_bounds:
            expected = self._zone_expected_points.get(zb.name, 0.0)
            actual_unique = len(self._zone_unique_rays.get(zb.name, set()))
            if 0.0 >= expected:
                result[f'{zb.name}_coverage_pct'] = 0.0
            else:
                result[f'{zb.name}_coverage_pct'] = 100.0 * actual_unique / expected
            result[f'{zb.name}_expected_rays'] = float(expected)
            result[f'{zb.name}_unique_rays'] = float(actual_unique)
        return result

    def shutdown(self) -> None:
        for s in self._zone_unique_rays.values():
            s.clear()
