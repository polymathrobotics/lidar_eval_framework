from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class DepthConsistency(MetricsBase):
    """Measures frame-to-frame stability of zone depth means.

    On a stationary scene the mean depth per zone should be identical every
    frame. Standard deviation across frames approaching zero is the ground
    truth. A non-zero std indicates the sensor is unstable between scans.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self._zone_depth_history: dict[str, list[float]] = {}
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        return

    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return

        self.pointcloud_by_zone = pointcloud_by_zone
        lidar_x = float(self.profiles.lidar_position[0])

        for zb in self.profiles.zone_bounds:
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                continue
            zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
            if len(zone_pts) == 0:
                continue
            mean_depth = float(np.mean(zone_pts[:, 0] - lidar_x))
            if zb.name not in self._zone_depth_history:
                self._zone_depth_history[zb.name] = []
            self._zone_depth_history[zb.name].append(mean_depth)

    def compute(self) -> dict[str, float]:
        result = {}
        for zone_name, depths in self._zone_depth_history.items():
            arr = np.array(depths)
            mean = float(np.mean(arr)) if len(arr) > 0 else 0.0
            std = float(np.std(arr)) if len(arr) >= 2 else 0.0
            result[f'{zone_name}_depth_temporal_std'] = std
            result[f'{zone_name}_depth_temporal_std_pct'] = (std / abs(mean) * 100.0) if mean != 0.0 else 0.0
            result[f'{zone_name}_depth_temporal_mean'] = mean
        return result

    def shutdown(self) -> None:
        self._zone_depth_history.clear()
