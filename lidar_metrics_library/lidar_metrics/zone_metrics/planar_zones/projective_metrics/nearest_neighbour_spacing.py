from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np
from scipy.spatial import cKDTree


class NearestNeighbourSpacing(MetricsBase):

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        # Per-scan NN stats keyed by output field — pooling raw points across
        # scans would inject zero-distance duplicates and corrupt the KD-tree.
        # Compute per scan, then average the per-scan stats at the end.
        self._scan_results: list[dict[str, float]] = []
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        self.outlier_thresh = self.config["lidar_metrics_parameters"]["nearest_neighbour_spacing"].get("outlier_threshold", 0.5)

    def update(self, pointcloud_by_zone) -> None:
        self.pointcloud_by_zone = pointcloud_by_zone

        if self.profiles is None:
            all_arrays = [a for a in pointcloud_by_zone.values() if len(a) > 0]
            union = np.vstack(all_arrays) if all_arrays else np.empty((0, 3))
            union = union[np.isfinite(union[:, :3]).all(axis=1)]
            self._scan_results.append(self._compute_for_points(union[:, :3], prefix=''))
            return

        scan_result = {}
        for zb in self.profiles.zone_bounds:
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                scan_result.update(self._compute_for_points(np.empty((0, 3)), prefix=f'{zb.name}_'))
                continue
            zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
            scan_result.update(self._compute_for_points(zone_pts[:, :3], prefix=f'{zb.name}_'))
        self._scan_results.append(scan_result)

    def compute(self) -> dict[str, float]:
        if not self._scan_results:
            return {}
        keys: set = set()
        for d in self._scan_results:
            keys.update(d.keys())
        result = {}
        for k in keys:
            vals = [d[k] for d in self._scan_results if k in d]
            result[k] = sum(vals) / len(vals) if vals else 0.0
        return result

    def _compute_for_points(self, pts: np.ndarray, prefix: str) -> dict[str, float]:
        n = pts.shape[0]

        if n < 2:
            return {
                f'{prefix}nn_p50': 0.0, f'{prefix}nn_p90': 0.0, f'{prefix}nn_p99': 0.0,
                f'{prefix}nn_mean': 0.0, f'{prefix}nn_outlier_frac': 0.0,
            }

        tree = cKDTree(pts)
        dists, _ = tree.query(pts, k=2)
        nn = dists[:, 1]

        return {
            f'{prefix}nn_p50': float(np.percentile(nn, 50)),
            f'{prefix}nn_p90': float(np.percentile(nn, 90)),
            f'{prefix}nn_p99': float(np.percentile(nn, 99)),
            f'{prefix}nn_mean': float(nn.mean()),
            f'{prefix}nn_outlier_frac': float((nn > self.outlier_thresh).mean()),
        }

    def shutdown(self) -> None:
        self._scan_results.clear()
