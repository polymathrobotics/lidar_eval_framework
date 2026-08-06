from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class SpatialDropout(MetricsBase):
    """Detects spatial dead zones in the sensor's scan pattern per zone.

    Divides each zone's y-z face into a grid of cells and tracks which cells
    receive at least one return across all frames. A cell that never receives
    a return across the entire test run is a dead zone. On a fully visible
    static scene, dropout fraction should be near zero. A sensor with dead
    pixels or blocked channels will show persistent empty cells.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self._zone_hit_grids: dict[str, np.ndarray] = {}
        self._zone_grid_meta: dict[str, tuple] = {}
        self._cell_size_m: float = 0.0
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        self._cell_size_m = float(self.config['lidar_metrics_parameters']['spatial_dropout']['cell_size_m'])
        if self.profiles is None:
            return
        for zb in self.profiles.zone_bounds:
            n_y = max(1, int(np.ceil((zb.y_max - zb.y_min) / self._cell_size_m)))
            n_z = max(1, int(np.ceil((zb.z_max - zb.z_min) / self._cell_size_m)))
            self._zone_hit_grids[zb.name] = np.zeros((n_y, n_z), dtype=bool)
            self._zone_grid_meta[zb.name] = (n_y, n_z, zb.y_min, zb.z_min)

    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return

        self.pointcloud_by_zone = pointcloud_by_zone

        for zb in self.profiles.zone_bounds:
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                continue
            zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
            if len(zone_pts) == 0:
                continue

            n_y, n_z, y_min, z_min = self._zone_grid_meta[zb.name]
            y_idx = np.floor((zone_pts[:, 1] - y_min) / self._cell_size_m).astype(int)
            z_idx = np.floor((zone_pts[:, 2] - z_min) / self._cell_size_m).astype(int)

            valid = (y_idx >= 0) & (y_idx < n_y) & (z_idx >= 0) & (z_idx < n_z)
            self._zone_hit_grids[zb.name][y_idx[valid], z_idx[valid]] = True

    def compute(self) -> dict[str, float]:
        result = {}
        for zone_name, grid in self._zone_hit_grids.items():
            total_cells = grid.size
            dropout_cells = total_cells - int(grid.sum())
            result[f'{zone_name}_dropout_frac'] = float(dropout_cells / total_cells) if total_cells > 0 else 0.0
            result[f'{zone_name}_dropout_cell_count'] = float(dropout_cells)
            result[f'{zone_name}_total_cell_count'] = float(total_cells)

            n_y, n_z, y_min, z_min = self._zone_grid_meta[zone_name]
            dead_positions = np.argwhere(~grid)
            for idx, (iy, iz) in enumerate(dead_positions):
                result[f'{zone_name}_dead_cell_{idx}_y_m'] = float(y_min + (iy + 0.5) * self._cell_size_m)
                result[f'{zone_name}_dead_cell_{idx}_z_m'] = float(z_min + (iz + 0.5) * self._cell_size_m)
        result['dead_cell_size_m'] = self._cell_size_m
        return result

    def shutdown(self) -> None:
        for grid in self._zone_hit_grids.values():
            grid[:] = False
