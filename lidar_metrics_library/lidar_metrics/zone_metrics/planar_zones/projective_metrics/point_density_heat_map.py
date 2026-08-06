import math

import numpy as np

from lidar_metrics.metric_interfaces.metrics_base import MetricsBase


class PointDensityHeatMapPerZone(MetricsBase):
    """For each zone, divide into grid cells of some small mm size
       genetate a rgb mapping

    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        # Angular resolutions are injected by the engine as attributes after
        # construction (see LidarMetricsEngine.create_plugins), so they're not
        # part of this constructor's signature.

        self.perfect_target_b = 0
        self.perfect_target_g = 255
        self.dense_target_r = 255
        self.dense_target_g = 0

        self.sparse_rgb_color_coding = (0, 0, 50)
        self.perfect_rgb_color_coding = (0, 255, 0)
        self.dense_rgb_color_coding = (255, 0, 0)

        self._cell_size_m: float = 0.0
        # {zone_name: {(iy, iz): expected_points}} — geometry-derived constants,
        # computed once in setup() since they only depend on the zone bounds and
        # lidar pose, not on any scan data.
        self._zone_cell_expected: dict[str, dict[tuple[int, int], float]] = {}
        # Running total of actual returns per cell across all scans.
        self._zone_cell_actual_counts: dict[str, np.ndarray] = {}
        self._scan_count: int = 0

        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        self._cell_size_m = float(self.config['lidar_metrics_parameters']['point_density_heat_map']['cell_size_m'])
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

            # Projective frustum shrinks the zone inward by these per side.
            # Cells outside the shrunken region are dropped — they're not part
            # of what this metric measures.
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

            self._zone_cell_expected[zb.name] = zone_cells
            # Full grid kept so np.add.at in update() can use fast indexed
            # binning; compute() only reads the cells present in zone_cells.
            self._zone_cell_actual_counts[zb.name] = np.zeros((n_y, n_z), dtype=np.int64)


    def rgb_mapping(self, density: float) -> tuple[int, int, int]:

        if density < 1:
            # Linear interpolation sparse (0,0,50) → perfect (0,255,0) as density goes 0→1.
            t = max(density, 0.0)
            sparse_b = self.sparse_rgb_color_coding[2]
            r = 0
            g = int(round(t * self.perfect_target_g))
            b = int(round((1.0 - t) * sparse_b + t * self.perfect_target_b))
            return (r, g, b)

        elif density > 1:
            if density >= 2:
                return self.dense_rgb_color_coding
            else:
                # Linear interpolation perfect (0,255,0) → dense (255,0,0) as density goes 1→2.
                t = density - 1.0
                r = int(round(t * self.dense_target_r))
                g = int(round((1.0 - t) * self.perfect_target_g + t * self.dense_target_g))
                b = 0
                return (r, g, b)

        else:
            return self.perfect_rgb_color_coding



    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return
        self.pointcloud_by_zone = pointcloud_by_zone
        self._scan_count += 1

        for zb in self.profiles.zone_bounds:
            counts_grid = self._zone_cell_actual_counts.get(zb.name)
            if counts_grid is None:
                continue
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                continue
            zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
            if len(zone_pts) == 0:
                continue

            n_y, n_z = counts_grid.shape
            y_idx = np.floor((zone_pts[:, 1] - zb.y_min) / self._cell_size_m).astype(int)
            z_idx = np.floor((zone_pts[:, 2] - zb.z_min) / self._cell_size_m).astype(int)
            valid = (y_idx >= 0) & (y_idx < n_y) & (z_idx >= 0) & (z_idx < n_z)
            np.add.at(counts_grid, (y_idx[valid], z_idx[valid]), 1)

    def compute(self) -> dict:
        if self.profiles is None or 0 == self._scan_count:
            return {}

        result: dict = {'cell_size_m': float(self._cell_size_m)}
        for zb in self.profiles.zone_bounds:
            counts = self._zone_cell_actual_counts.get(zb.name)
            expected_cells = self._zone_cell_expected.get(zb.name)
            if counts is None or expected_cells is None:
                continue

            # One entry per in-bound cell: physical (y, z) center + rgb. Cells
            # outside the projective window aren't in expected_cells, so they're
            # automatically excluded.
            cells_out: list[dict] = []
            for (iy, iz), expected in expected_cells.items():
                if 0.0 >= expected:
                    continue
                density = float(counts[iy, iz]) / (self._scan_count * expected)
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
        for counts in self._zone_cell_actual_counts.values():
            counts[:] = 0
        self._scan_count = 0
