from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class PointYieldPerZone(MetricsBase):
    """Reports point count and yield fraction broken out per zone.

    Unlike the aggregate PointYieldRate, this reveals which specific zone
    is losing returns — e.g. the green wall absorbing more laser energy
    than the whiteboard and dropping points.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self._zone_finite_counts: dict[str, int] = {}
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
                self._zone_finite_counts.setdefault(zb.name, 0)
                continue
            n_finite = int(np.isfinite(zone_pts[:, :3]).all(axis=1).sum())
            self._zone_finite_counts[zb.name] = self._zone_finite_counts.get(zb.name, 0) + n_finite

    def compute(self) -> dict[str, float]:
        if self.profiles is None:
            return {}

        total_valid = sum(self._zone_finite_counts.values())

        result = {}
        for zb in self.profiles.zone_bounds:
            count = self._zone_finite_counts.get(zb.name, 0)
            result[f'{zb.name}_point_count'] = float(count)
            result[f'{zb.name}_yield_frac'] = float(count / total_valid) if total_valid > 0 else 0.0

        return result

    def shutdown(self) -> None:
        self._zone_finite_counts.clear()
