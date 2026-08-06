from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class SurfaceFitResidual(MetricsBase):
    """Fits a plane to each zone's points via PCA and reports residuals.

    Also reports the X offset between the fitted plane center and the expected
    x_surface from TF — a non-zero offset means the sensor sees the surface
    at a different position than the TF tree says it should be.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        # Raw point chunks per zone, concatenated and PCA-fit once in compute().
        # Fitting on the union of all scans yields a tighter plane estimate than
        # averaging per-scan fits, and is correct for a static scene.
        self._zone_point_chunks: dict[str, list[np.ndarray]] = {}
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
                continue
            zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)][:, :3]
            if len(zone_pts) == 0:
                continue
            self._zone_point_chunks.setdefault(zb.name, []).append(zone_pts.astype(np.float64, copy=False))

    def compute(self) -> dict[str, float]:
        if self.profiles is None:
            return {}

        result = {}

        for zb in self.profiles.zone_bounds:
            # Expected zone plane straight from the profile bounds (full zone, no
            # padding applied). Emitted for every zone regardless of returns since
            # it depends only on the profile, not the point data.
            viz = {
                'expected_x': float(zb.x_surface),
                'expected_y_min': float(zb.y_min),
                'expected_y_max': float(zb.y_max),
                'expected_z_min': float(zb.z_min),
                'expected_z_max': float(zb.z_max),
            }

            chunks = self._zone_point_chunks.get(zb.name)
            if not chunks:
                result[f'{zb.name}_fit_rms'] = 0.0
                result[f'{zb.name}_fit_x_offset'] = 0.0
                result[f'{zb.name}_visualization'] = viz
                continue
            zone_pts = np.vstack(chunks)
            if len(zone_pts) < 3:
                result[f'{zb.name}_fit_rms'] = 0.0
                result[f'{zb.name}_fit_x_offset'] = 0.0
                result[f'{zb.name}_visualization'] = viz
                continue

            mu = zone_pts.mean(axis=0)
            X = zone_pts - mu
            C = (X.T @ X) / float(len(zone_pts))

            # Smallest eigenvector = surface normal
            w, v = np.linalg.eigh(C)
            normal = v[:, 0]

            # RMS point-to-plane distance
            residuals = X @ normal
            rms = float(np.sqrt(np.mean(residuals ** 2)))

            # How far the fitted plane center is from TF's expected x_surface
            x_offset = float(mu[0] - zb.x_surface)

            result[f'{zb.name}_fit_rms'] = rms
            result[f'{zb.name}_fit_x_offset'] = abs(x_offset)

            # Fitted PCA plane alongside the expected plane. Zone-prefixed so the
            # report pivot files it under this zone (the sub-keys are unprefixed —
            # they end up under <zone>/SurfaceFitResidual/visualization).
            viz.update({
                'plane_center_x': float(mu[0]),
                'plane_center_y': float(mu[1]),
                'plane_center_z': float(mu[2]),
                'plane_normal_x': float(normal[0]),
                'plane_normal_y': float(normal[1]),
                'plane_normal_z': float(normal[2]),
                'plane_bounds_y_min': float(zb.y_min),
                'plane_bounds_y_max': float(zb.y_max),
                'plane_bounds_z_min': float(zb.z_min),
                'plane_bounds_z_max': float(zb.z_max),
            })
            result[f'{zb.name}_visualization'] = viz

        return result

    def shutdown(self) -> None:
        self._zone_point_chunks.clear()
