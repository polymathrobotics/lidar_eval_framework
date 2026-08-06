from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class PenetrationDropoutRate(MetricsBase):
    """Computes the dropout rate based on rays aimed at each zone vs the points
    that actually landed inside the zone's spatial bounding box.

    Under the per-zone refactor, `self.pointcloud_by_zone[zb.name]` is the set
    of points aimed at this zone's angular window (the projective filter result).
    We derive "valid" points by checking which of those landed inside the zone's
    3D bbox. The ratio is `valid / total_rays` per zone.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        return

    def update(self, pointcloud_by_zone) -> None:
        self.pointcloud_by_zone = pointcloud_by_zone

    def compute(self) -> dict[str, float]:
        if self.profiles is None:
            return {}

        result = {}

        for zb in self.profiles.zone_bounds:
            zone_pts = self.pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                result[f'{zb.name}_valid_point_count'] = 0.0
                result[f'{zb.name}_total_point_count'] = 0.0
                result[f'{zb.name}_ray_count'] = 0.0
                result[f'{zb.name}_valid_point_rate'] = 0.0
                continue

            zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
            ray_count = float(len(zone_pts))

            # "Valid" points are rays that returned inside the zone's spatial bbox
            # (i.e., hit the planar surface rather than penetrating past or dropping out).
            in_bbox_mask = (
                (zone_pts[:, 0] >= zb.x_min) & (zone_pts[:, 0] <= zb.x_max)
                & (zone_pts[:, 1] >= zb.y_min) & (zone_pts[:, 1] <= zb.y_max)
                & (zone_pts[:, 2] >= zb.z_min) & (zone_pts[:, 2] <= zb.z_max)
            )
            valid_count = float(int(in_bbox_mask.sum()))

            result[f'{zb.name}_valid_point_count'] = valid_count
            result[f'{zb.name}_total_point_count'] = ray_count
            result[f'{zb.name}_ray_count'] = ray_count
            result[f'{zb.name}_valid_point_rate'] = valid_count / ray_count if ray_count > 0 else 0.0

        return result

    def shutdown(self) -> None:
        return
