from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
from .zone_utils import points_in_noise_region
import numpy as np


class NoiseRegionContamination(MetricsBase):
    """Measures the fraction of in-zone points falling inside known noise regions.

    Noise regions are the edge between whiteboard/wall and the wall corner —
    areas prone to multipath returns. A high contamination fraction indicates
    the sensor is producing more spurious returns at geometry boundaries than
    expected.

    Operates on the union of per-zone clouds. Noise regions that extend outside
    any zone's mask are only partially observed by this metric — see the note
    in zone_utils.points_in_noise_region for context.
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

        all_arrays = [a for a in self.pointcloud_by_zone.values() if len(a) > 0]
        if not all_arrays:
            return {'noise_contamination_frac': 0.0}

        pts = np.vstack(all_arrays)
        pts = pts[np.isfinite(pts[:, :3]).all(axis=1)]
        n_total = len(pts)

        if n_total == 0:
            return {'noise_contamination_frac': 0.0}

        combined_mask = np.zeros(n_total, dtype=bool)
        result = {}

        for nr in self.profiles.noise_regions:
            mask = points_in_noise_region(pts, nr)
            combined_mask |= mask
            result[f'{nr.name}_count'] = float(mask.sum())

        result['noise_contamination_frac'] = float(combined_mask.sum() / n_total)

        return result

    def shutdown(self) -> None:
        return
