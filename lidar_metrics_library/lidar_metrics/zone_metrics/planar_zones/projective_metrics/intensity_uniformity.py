from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class IntensityUniformity(MetricsBase):
    """Measures spatial uniformity of intensity returns across each zone.

    On a uniform-colour surface, intensity should be consistent regardless of
    where in the zone the beam lands. The zone is divided into spatial bins
    along the y axis and the coefficient of variation (CV) of bin means is
    computed. A linear regression slope across y is also reported.

    CV near zero and slope near zero = uniform sensor response.
    High CV or large slope = edge falloff or non-uniform sensitivity —
    a deployment risk for detecting low-reflectivity objects near the FOV edges.
    """

    _N_BINS: int = 5  # spatial bins along y per zone

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
                result[f'{zb.name}_intensity_cv'] = 0.0
                result[f'{zb.name}_intensity_slope'] = 0.0
                continue

            zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]

            if len(zone_pts) < self._N_BINS * 2:
                result[f'{zb.name}_intensity_cv'] = 0.0
                result[f'{zb.name}_intensity_slope'] = 0.0
                continue

            y_vals = zone_pts[:, 1]
            intensities = zone_pts[:, 3]

            # CV of intensity means across spatial bins along y
            bin_edges = np.linspace(zb.y_min, zb.y_max, self._N_BINS + 1)
            bin_means = []
            for i in range(self._N_BINS):
                in_bin = (y_vals >= bin_edges[i]) & (y_vals < bin_edges[i + 1])
                if in_bin.sum() > 0:
                    bin_means.append(float(np.mean(intensities[in_bin])))

            if len(bin_means) >= 2:
                bm = np.array(bin_means)
                mean_bm = float(np.mean(bm))
                cv = float(np.std(bm) / mean_bm) if mean_bm > 0.0 else 0.0
            else:
                cv = 0.0

            # Linear intensity gradient across y (slope close to 0 = uniform)
            slope = float(np.polyfit(y_vals, intensities, 1)[0]) if len(y_vals) >= 2 else 0.0

            result[f'{zb.name}_intensity_cv'] = cv
            result[f'{zb.name}_intensity_slope'] = slope

        return result

    def shutdown(self) -> None:
        return
