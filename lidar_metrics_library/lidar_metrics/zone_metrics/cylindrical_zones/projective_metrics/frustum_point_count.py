from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class FrustumPointCount(MetricsBase):
    """Counts projective (frustum-filtered) returns per cylindrical zone.

    Simple yield indicator: per scan it counts the finite points the projective
    filter kept for each zone, accumulates over the run, and reports the total,
    the mean per scan, and the scan count per zone.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        # zone -> {total_points, n_scans}
        self._counts: dict[str, dict[str, int]] = {}
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        return

    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return
        self.pointcloud_by_zone = pointcloud_by_zone

        for zb in self.profiles.zone_bounds:
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                count = 0
            else:
                count = int(np.isfinite(zone_pts[:, :3]).all(axis=1).sum())

            entry = self._counts.setdefault(zb.name, {'total_points': 0, 'n_scans': 0})
            entry['total_points'] += count
            entry['n_scans'] += 1

    def compute(self) -> dict[str, float]:
        result: dict[str, float] = {}
        for zone, entry in self._counts.items():
            n_scans = entry['n_scans']
            total = entry['total_points']
            result[f'{zone}_total_points'] = float(total)
            result[f'{zone}_mean_points_per_scan'] = float(total) / n_scans if n_scans else 0.0
            result[f'{zone}_n_scans'] = float(n_scans)
        return result

    def shutdown(self) -> None:
        self._counts.clear()
