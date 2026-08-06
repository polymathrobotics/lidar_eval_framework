from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class RangeConsistency(MetricsBase):
    """Measures frame-to-frame stability of per-zone 3D range means.

    On a stationary scene the mean 3D distance from the lidar to each zone
    should be identical every frame. Standard deviation across frames
    approaching zero is the ground truth. A non-zero std indicates ranging
    instability — critical for SLAM and obstacle detection reliability.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self._zone_range_history: dict[str, list[float]] = {}
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        return

    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return

        self.pointcloud_by_zone = pointcloud_by_zone
        lidar_pos = self.profiles.lidar_position

        for zb in self.profiles.zone_bounds:
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                continue
            zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
            if len(zone_pts) == 0:
                continue
            diff = zone_pts[:, :3] - lidar_pos
            ranges = np.sqrt((diff ** 2).sum(axis=1))
            if zb.name not in self._zone_range_history:
                self._zone_range_history[zb.name] = []
            self._zone_range_history[zb.name].append(float(np.mean(ranges)))

    def compute(self) -> dict[str, float]:
        result = {}
        for zone_name, ranges in self._zone_range_history.items():
            arr = np.array(ranges)
            mean = float(np.mean(arr)) if len(arr) > 0 else 0.0
            std = float(np.std(arr)) if len(arr) >= 2 else 0.0
            result[f'{zone_name}_range_precision_std'] = std
            result[f'{zone_name}_range_precision_mean'] = mean
            result[f'{zone_name}_range_precision_std_pct'] = (std / abs(mean) * 100.0) if mean != 0.0 else 0.0
        return result

    def shutdown(self) -> None:
        self._zone_range_history.clear()
