from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np

class LocalGeometryDegeneracy(MetricsBase):

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        return

    def update(self, pointcloud_by_zone) -> None:
        self.pointcloud_by_zone = pointcloud_by_zone

    def compute(self) -> dict[str, float]:
        # Operates on the union of all per-zone clouds.
        all_arrays = [a for a in self.pointcloud_by_zone.values() if len(a) > 0]
        if not all_arrays:
            return self._empty_result()

        pts = np.vstack(all_arrays).astype(np.float64)[:, :3]
        pts = pts[np.isfinite(pts).all(axis=1)]
        n = pts.shape[0]
        if n < 3:
            return self._empty_result()

        # center
        mu = pts.mean(axis=0)
        X = pts - mu

        # covariance (3x3)
        C = (X.T @ X) / float(n)

        # eigenvalues (ascending from eigvalsh)
        w = np.linalg.eigvalsh(C)
        eig3, eig2, eig1 = float(w[0]), float(w[1]), float(w[2])  # eig1 >= eig2 >= eig3

        if eig1 <= 0.0:
            return {
                "linearity": 0.0,
                "planarity": 0.0,
                "scattering": 0.0,
                "eig1": eig1,
                "eig2": eig2,
                "eig3": eig3,
            }

        linearity = (eig1 - eig2) / eig1
        planarity = (eig2 - eig3) / eig1
        scattering = eig3 / eig1

        return {
            "linearity": float(linearity),
            "planarity": float(planarity),
            "scattering": float(scattering),
            "eig1": eig1,
            "eig2": eig2,
            "eig3": eig3,
        }

    def _empty_result(self) -> dict[str, float]:
        return {
            "linearity": 0.0,
            "planarity": 0.0,
            "scattering": 0.0,
            "eig1": 0.0,
            "eig2": 0.0,
            "eig3": 0.0,
        }

    def shutdown(self) -> None:
        return
