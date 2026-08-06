import math

from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class RaySurfaceRangeError(MetricsBase):
    """Per-cell ray-vs-surface 3D range-error heat map over a cylindrical zone.

    For each return P, the ray from the lidar L through P is intersected with
    the ideal cylinder (radius r about the vertical axis). The near intersection
    S is where that ray *should* meet the surface, so the per-point error is the
    full 3D euclidean distance ||P - S|| — equivalently the along-ray range error
    (P, S, L are colinear).

    The surface is gridded into azimuth x height cells (binned by S, so cells
    are true surface patches). update() records the per-scan mean error per
    cell; compute() averages across scans and color-codes each cell
    (green = on-surface, red = large range error). Visualization metric — emits
    a per-zone list of cells.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self.accurate_rgb_color_coding = (0, 255, 0)
        self.inaccurate_rgb_color_coding = (255, 0, 0)
        # zone -> { (az_idx, z_idx): {err_sum, n, sx, sy, sz} }
        self._acc: dict[str, dict[tuple[int, int], dict]] = {}
        self._cell_size_m = 0.05
        self.step_size_m = 0.02  # 3D range error (m) that maps to fully red
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        params = self.config.get('lidar_metrics_parameters', {}).get('ray_surface_range_error', {})
        self._cell_size_m = float(params.get('cell_size_m', 0.05))
        self.step_size_m = float(params.get('step_size_m', 0.02))

    def rgb_mapping(self, euclidean_distance: float) -> tuple[int, int, int]:
        if self.step_size_m <= 0.0:
            return self.accurate_rgb_color_coding
        steps = euclidean_distance / self.step_size_m
        r_value = min(int(steps * self.inaccurate_rgb_color_coding[0]), self.inaccurate_rgb_color_coding[0])
        g_value = max(self.accurate_rgb_color_coding[1] - int(steps * self.accurate_rgb_color_coding[1]), 0)
        b_value = 0
        return (r_value, g_value, b_value)

    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return
        self.pointcloud_by_zone = pointcloud_by_zone

        lidar = np.asarray(self.profiles.lidar_position, dtype=np.float64)
        lx, ly = float(lidar[0]), float(lidar[1])

        for zb in self.profiles.zone_bounds:
            radius = float(zb.radius)
            if radius <= 0.0:
                continue

            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                continue
            pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)][:, :3].astype(np.float64)
            if len(pts) == 0:
                continue

            # Ray L -> P, parameterised so t=1 lands on P. Intersect the vertical
            # cylinder in the XY plane: |(L_xy - center) + t*(P_xy - L_xy)| = r.
            ray = pts - lidar
            ex = lx - zb.center_x
            ey = ly - zb.center_y
            a = ray[:, 0] ** 2 + ray[:, 1] ** 2
            b = 2.0 * (ex * ray[:, 0] + ey * ray[:, 1])
            c = ex * ex + ey * ey - radius * radius
            disc = b * b - 4.0 * a * c

            with np.errstate(invalid='ignore', divide='ignore'):
                sqrt_disc = np.sqrt(np.where(disc >= 0.0, disc, np.nan))
                t_near = (-b - sqrt_disc) / (2.0 * a)   # smaller root = near face
                t_far = (-b + sqrt_disc) / (2.0 * a)
            # Prefer the near intersection in front of the lidar; fall back to far.
            t = np.where(t_near > 0.0, t_near, t_far)
            valid = (a > 0.0) & (disc >= 0.0) & np.isfinite(t) & (t > 0.0)
            if not np.any(valid):
                continue

            pv = pts[valid]
            surface = lidar + t[valid][:, None] * ray[valid]   # expected surface point S
            err = np.linalg.norm(pv - surface, axis=1)         # 3D euclidean = |1 - t| * ||P - L||

            # Bin by the expected on-surface point so cells are true surface patches.
            theta = np.arctan2(surface[:, 1] - zb.center_y, surface[:, 0] - zb.center_x)
            ref = math.atan2(ly - zb.center_y, lx - zb.center_x)
            rel = np.arctan2(np.sin(theta - ref), np.cos(theta - ref))
            d_theta = self._cell_size_m / radius
            az_idx = np.floor(rel / d_theta).astype(np.int64)
            z_idx = np.floor((surface[:, 2] - zb.z_min) / self._cell_size_m).astype(np.int64)

            cells = np.stack([az_idx, z_idx], axis=1)
            uniq, inv = np.unique(cells, axis=0, return_inverse=True)

            zone_acc = self._acc.setdefault(zb.name, {})
            for k, (ai, zi) in enumerate(uniq):
                mask = inv == k
                cell_err = float(err[mask].mean())
                cell_s = surface[mask].mean(axis=0)
                entry = zone_acc.setdefault(
                    (int(ai), int(zi)),
                    {'err_sum': 0.0, 'n': 0, 'sx': 0.0, 'sy': 0.0, 'sz': 0.0},
                )
                entry['err_sum'] += cell_err
                entry['n'] += 1
                entry['sx'] += float(cell_s[0])
                entry['sy'] += float(cell_s[1])
                entry['sz'] += float(cell_s[2])

    def compute(self) -> dict:
        # Keyed by zone name so the report pivot files the cell list under the
        # zone (<zone>/RaySurfaceRangeError).
        result: dict = {}
        for zone, cells in self._acc.items():
            cell_list = []
            for (az_idx, z_idx), entry in cells.items():
                n = entry['n']
                avg_err = entry['err_sum'] / n
                cell_list.append({
                    'az_index': az_idx,
                    'z_index': z_idx,
                    'expected_surface_point': [entry['sx'] / n, entry['sy'] / n, entry['sz'] / n],
                    'avg_surface_error': avg_err,
                    'rgb': list(self.rgb_mapping(avg_err)),
                    'n_scans': n,
                })
            result[zone] = cell_list
        return result

    def shutdown(self) -> None:
        self._acc.clear()
