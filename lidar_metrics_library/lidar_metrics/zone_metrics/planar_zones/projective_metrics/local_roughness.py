from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np
from scipy.spatial import cKDTree


class LocalRoughnessKNN(MetricsBase):

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        # KD-tree on pooled points would skew k-nearest-neighbour structure
        # because duplicates from each scan collapse k onto themselves. Compute
        # roughness per scan, average per-key at end.
        self._scan_results: list[dict[str, float]] = []
        self.k = 30
        self.sample_size = None  # e.g. 10000 to speed up
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        self.k = int(self.config["lidar_metrics_parameters"]["local_roughness_knn"].get("k"))
        self.sample_size = int(self.config["lidar_metrics_parameters"]["local_roughness_knn"].get("sample_size"))

    def update(self, pointcloud_by_zone) -> None:
        self.pointcloud_by_zone = pointcloud_by_zone

        if self.profiles is None:
            all_pts = self._union_xyz()
            self._scan_results.append(self._compute_for_points(all_pts, prefix=''))
            return

        scan_result = {}
        for zb in self.profiles.zone_bounds:
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                scan_result.update(self._compute_for_points(np.empty((0, 3)), prefix=f'{zb.name}_'))
                continue
            zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
            scan_result.update(self._compute_for_points(zone_pts[:, :3].astype(np.float64), prefix=f'{zb.name}_'))
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

    def _union_xyz(self) -> np.ndarray:
        all_arrays = [arr for arr in self.pointcloud_by_zone.values() if len(arr) > 0]
        if not all_arrays:
            return np.empty((0, 3))
        union = np.vstack(all_arrays).astype(np.float64)
        union = union[np.isfinite(union[:, :3]).all(axis=1)]
        return union[:, :3]

    def _compute_for_points(self, pts: np.ndarray, prefix: str) -> dict[str, float]:
        n = pts.shape[0]
        k = self.k

        if n < k + 1:
            return {
                f'{prefix}rough_p50': 0.0,
                f'{prefix}rough_p90': 0.0,
                f'{prefix}rough_p99': 0.0,
                f'{prefix}rough_mean': 0.0,
                f'{prefix}rough_std': 0.0,
                f'{prefix}rough_n': 0.0,
            }

        if self.sample_size is not None and n > self.sample_size:
            idx = np.random.choice(n, size=self.sample_size, replace=False)
            query_pts = pts[idx]
        else:
            query_pts = pts

        tree = cKDTree(pts)
        _, nn_idx = tree.query(query_pts, k=k + 1)
        nn_idx = nn_idx[:, 1:]

        rough = np.empty(query_pts.shape[0], dtype=np.float64)
        for i in range(query_pts.shape[0]):
            nbrs = pts[nn_idx[i]]
            mu = nbrs.mean(axis=0)
            X = nbrs - mu
            C = (X.T @ X) / float(k)
            w = np.linalg.eigvalsh(C)
            rough[i] = np.sqrt(max(float(w[0]), 0.0))

        return {
            f'{prefix}rough_p50': float(np.percentile(rough, 50)),
            f'{prefix}rough_p90': float(np.percentile(rough, 90)),
            f'{prefix}rough_p99': float(np.percentile(rough, 99)),
            f'{prefix}rough_mean': float(rough.mean()),
            f'{prefix}rough_std': float(rough.std(ddof=0)),
            f'{prefix}rough_n': float(rough.shape[0]),
        }

    def shutdown(self) -> None:
        self._scan_results.clear()
