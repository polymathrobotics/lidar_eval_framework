import math

import numpy as np

from lidar_metrics.metric_interfaces.metrics_base import MetricsBase


class PointDensityHeatMapOverBag(MetricsBase):
    """Per-cell coverage heat map of the lidar's advertised angular grid,
    accumulated over every scan in the bag.

    Same idea as PointDensityCoverageOverBag, but at cell granularity:
    for each y/z grid cell on the zone surface, count unique rays (binned at
    the lidar's Δθ_h × Δθ_v) that landed in that cell across all scans,
    compare to the geometric expected ray count per cell, color via
    rgb_mapping. Padding-ring cells are dropped (not in the projective window).

    Difference vs. PointDensityHeatMapPerZone:
      - PointDensityHeatMapPerZone averages per-scan densities per cell
        (per-frame question: "did this cell get its quota each scan?").
      - This one deduplicates rays across the bag (per-bag question: "did the
        lidar ever fill the cell's advertised ray budget?"). Robust to
        interlaced scanners that need multiple frames to complete a pattern.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self.perfect_target_b = 0
        self.perfect_target_g = 255
        self.dense_target_r = 255
        self.dense_target_g = 0

        self.sparse_rgb_color_coding = (0, 0, 50)
        self.perfect_rgb_color_coding = (0, 255, 0)
        self.dense_rgb_color_coding = (255, 0, 0)

        self._cell_size_m: float = 0.0
        # {zone_name: {(iy, iz): expected_rays}} — geometry-only, set in setup.
        self._zone_cell_expected: dict[str, dict[tuple[int, int], float]] = {}
        # {zone_name: {(iy, iz): set of packed (az_bin, el_bin) int64s}} — only
        # in-bound cells get an entry, mirroring _zone_cell_expected.
        self._zone_cell_unique_rays: dict[str, dict[tuple[int, int], set]] = {}

        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        self._cell_size_m = float(
            self.config['lidar_metrics_parameters']['point_density_heat_map_over_bag']['cell_size_m']
        )
        if self.profiles is None:
            return

        lidar_pos = self.profiles.lidar_position
        h_rad = math.radians(self.horizontal_resolution)
        v_rad = math.radians(self.vertical_resolution)
        solid_angle_per_beam = h_rad * v_rad
        cell_area = self._cell_size_m * self._cell_size_m

        for zb in self.profiles.zone_bounds:
            n_y = max(1, int(np.ceil((zb.y_max - zb.y_min) / self._cell_size_m)))
            n_z = max(1, int(np.ceil((zb.z_max - zb.z_min) / self._cell_size_m)))
            zone_cells: dict[tuple[int, int], float] = {}
            zone_ray_sets: dict[tuple[int, int], set] = {}

            y_min_p = zb.y_min + float(zb.y_padding)
            y_max_p = zb.y_max - float(zb.y_padding)
            z_min_p = zb.z_min + float(zb.z_padding)
            z_max_p = zb.z_max - float(zb.z_padding)

            for iy in range(n_y):
                for iz in range(n_z):
                    y_c = zb.y_min + (iy + 0.5) * self._cell_size_m
                    z_c = zb.z_min + (iz + 0.5) * self._cell_size_m
                    if not (y_min_p <= y_c <= y_max_p and z_min_p <= z_c <= z_max_p):
                        continue

                    dx = float(zb.x_surface - lidar_pos[0])
                    dy = float(y_c - lidar_pos[1])
                    dz = float(z_c - lidar_pos[2])
                    d_sq = dx * dx + dy * dy + dz * dz
                    if 0.0 == d_sq or 0.0 == solid_angle_per_beam:
                        continue

                    d = math.sqrt(d_sq)
                    cos_alpha = abs(dx) / d
                    zone_cells[(iy, iz)] = (cell_area / (d_sq * solid_angle_per_beam)) * cos_alpha
                    zone_ray_sets[(iy, iz)] = set()

            self._zone_cell_expected[zb.name] = zone_cells
            self._zone_cell_unique_rays[zb.name] = zone_ray_sets

    def rgb_mapping(self, density: float) -> tuple[int, int, int]:
        if density < 1:
            t = max(density, 0.0)
            sparse_b = self.sparse_rgb_color_coding[2]
            r = 0
            g = int(round(t * self.perfect_target_g))
            b = int(round((1.0 - t) * sparse_b + t * self.perfect_target_b))
            return (r, g, b)
        elif density > 1:
            if density >= 2:
                return self.dense_rgb_color_coding
            t = density - 1.0
            r = int(round(t * self.dense_target_r))
            g = int(round((1.0 - t) * self.perfect_target_g + t * self.dense_target_g))
            b = 0
            return (r, g, b)
        return self.perfect_rgb_color_coding

    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return
        self.pointcloud_by_zone = pointcloud_by_zone

        h_rad = math.radians(self.horizontal_resolution)
        v_rad = math.radians(self.vertical_resolution)
        if 0.0 >= h_rad or 0.0 >= v_rad or 0.0 >= self._cell_size_m:
            return

        lidar_pos = self.profiles.lidar_position

        for zb in self.profiles.zone_bounds:
            cell_ray_sets = self._zone_cell_unique_rays.get(zb.name)
            if cell_ray_sets is None or len(cell_ray_sets) == 0:
                continue
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or 0 == len(zone_pts):
                continue
            finite = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
            if 0 == len(finite):
                continue

            dx = finite[:, 0] - lidar_pos[0]
            dy = finite[:, 1] - lidar_pos[1]
            dz = finite[:, 2] - lidar_pos[2]
            az = np.arctan2(dy, dx)
            el = np.arctan2(dz, np.sqrt(dx * dx + dy * dy))
            az_bin = np.floor(az / h_rad).astype(np.int64)
            el_bin = np.floor(el / v_rad).astype(np.int64)
            ray_id = (az_bin << 32) | (el_bin & 0xFFFFFFFF)

            y_idx = np.floor((finite[:, 1] - zb.y_min) / self._cell_size_m).astype(np.int64)
            z_idx = np.floor((finite[:, 2] - zb.z_min) / self._cell_size_m).astype(np.int64)

            for iy, iz, rid in zip(y_idx.tolist(), z_idx.tolist(), ray_id.tolist()):
                cell_set = cell_ray_sets.get((iy, iz))
                if cell_set is not None:
                    cell_set.add(int(rid))

    def compute(self) -> dict:
        if self.profiles is None:
            return {}

        result: dict = {'cell_size_m': float(self._cell_size_m)}
        for zb in self.profiles.zone_bounds:
            expected_cells = self._zone_cell_expected.get(zb.name)
            cell_ray_sets = self._zone_cell_unique_rays.get(zb.name)
            if expected_cells is None or cell_ray_sets is None:
                continue

            cells_out: list[dict] = []
            for (iy, iz), expected in expected_cells.items():
                if 0.0 >= expected:
                    continue
                actual_unique = len(cell_ray_sets.get((iy, iz), set()))
                density = actual_unique / expected
                r, g, b = self.rgb_mapping(density)
                y_center = zb.y_min + (iy + 0.5) * self._cell_size_m
                z_center = zb.z_min + (iz + 0.5) * self._cell_size_m
                cells_out.append({
                    'y_center': float(y_center),
                    'z_center': float(z_center),
                    'rgb': [int(r), int(g), int(b)],
                })

            result[zb.name] = cells_out
        return result

    def shutdown(self) -> None:
        for sets in self._zone_cell_unique_rays.values():
            for s in sets.values():
                s.clear()
