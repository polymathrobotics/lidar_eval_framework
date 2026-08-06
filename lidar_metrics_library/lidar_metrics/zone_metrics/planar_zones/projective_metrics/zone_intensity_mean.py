from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np
from itertools import combinations


class ZoneIntensityMean(MetricsBase):
    """Measures per-zone intensity mean and pairwise intensity ratios between zones.

    Absolute intensity values are sensor- and distance-dependent, so they are
    reported as informational only. The physically meaningful comparison is the
    ratio between zones — a white surface should always be brighter than turquoise
    regardless of sensor scale or range. The expected ratio is derived from the
    relative reflectivity values in KNOWN_COLORS, which are scale-agnostic.
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

        zone_means: dict[str, float] = {}
        zone_expected: dict[str, float] = {}
        result = {}

        for zb in self.profiles.zone_bounds:
            zone_pts = self.pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                result[f'{zb.name}_intensity_mean'] = 0.0
                result[f'{zb.name}_intensity_std'] = 0.0
                zone_means[zb.name] = 0.0
            else:
                zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
                if len(zone_pts) == 0:
                    result[f'{zb.name}_intensity_mean'] = 0.0
                    result[f'{zb.name}_intensity_std'] = 0.0
                    zone_means[zb.name] = 0.0
                else:
                    intensities = zone_pts[:, 3]
                    mean_i = float(np.mean(intensities))
                    result[f'{zb.name}_intensity_mean'] = mean_i
                    result[f'{zb.name}_intensity_std'] = float(np.std(intensities))
                    zone_means[zb.name] = mean_i

            zone_expected[zb.name] = zb.zone_config.expected_intensity

        # Pairwise intensity ratios — sensor-scale-agnostic comparison
        zone_names = list(zone_means.keys())
        for name_a, name_b in combinations(zone_names, 2):
            mean_a = zone_means[name_a]
            mean_b = zone_means[name_b]
            exp_a = zone_expected[name_a]
            exp_b = zone_expected[name_b]

            measured_ratio = mean_a / mean_b if mean_b > 0.0 else 0.0
            expected_ratio = exp_a / exp_b if exp_b > 0.0 else 0.0

            key = f'{name_a}_to_{name_b}'
            result[f'{key}_intensity_ratio'] = measured_ratio
            result[f'{key}_expected_ratio'] = expected_ratio
            result[f'{key}_ratio_error'] = measured_ratio - expected_ratio

        return result

    def shutdown(self) -> None:
        return
