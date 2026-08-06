from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class AverageSpatialDropout(MetricsBase):
    """Per-frame spatial dropout averaged across all frames.

    For each zone, grids the y-z face into cells and counts how many frames
    each cell received at least one return. Reports the mean miss rate
    across cells, which equals the expected fraction of cells missed by any
    single scan. Unlike SpatialDropout (which only flags permanently dead
    cells), this captures scan-pattern jitter: cells that are hit some
    frames and missed others contribute proportionally to the dropout.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self._zone_hit_counts: dict[str, np.ndarray] = {}
        self._zone_grid_meta: dict[str, tuple] = {}
        self._frame_count: int = 0
        self._cell_size_m: float = 0.0
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        self._cell_size_m = float(self.config['lidar_metrics_parameters']['spatial_dropout']['cell_size_m'])
        if self.profiles is None:
            return
        for zb in self.profiles.zone_bounds:
            n_y = max(1, int(np.ceil((zb.y_max - zb.y_min) / self._cell_size_m)))
            n_z = max(1, int(np.ceil((zb.z_max - zb.z_min) / self._cell_size_m)))
            self._zone_hit_counts[zb.name] = np.zeros((n_y, n_z), dtype=np.int64)
            self._zone_grid_meta[zb.name] = (n_y, n_z, zb.y_min, zb.z_min)

    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return

        self.pointcloud_by_zone = pointcloud_by_zone

        # Count this frame even if a zone has zero points — that frame contributes 100% dropout for that zone.
        self._frame_count += 1

        for zb in self.profiles.zone_bounds:
            n_y, n_z, y_min, z_min = self._zone_grid_meta[zb.name]
            frame_hits = np.zeros((n_y, n_z), dtype=bool)

            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is not None and len(zone_pts) > 0:
                zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
                if len(zone_pts) > 0:
                    y_idx = np.floor((zone_pts[:, 1] - y_min) / self._cell_size_m).astype(int)
                    z_idx = np.floor((zone_pts[:, 2] - z_min) / self._cell_size_m).astype(int)
                    valid = (y_idx >= 0) & (y_idx < n_y) & (z_idx >= 0) & (z_idx < n_z)
                    frame_hits[y_idx[valid], z_idx[valid]] = True

            # A cell hit by multiple points in the same frame still counts as one hit for that frame.
            self._zone_hit_counts[zb.name] += frame_hits.astype(np.int64)

    def compute(self) -> dict[str, float]:
        result = {}
        for zone_name, counts in self._zone_hit_counts.items():
            total_cells = counts.size
            if 0 == self._frame_count or 0 == total_cells:
                result[f'{zone_name}_avg_dropout_frac'] = 0.0
                result[f'{zone_name}_mean_cell_hit_rate'] = 0.0
                continue
            mean_hit_rate = float(counts.sum()) / float(self._frame_count * total_cells)
            result[f'{zone_name}_avg_dropout_frac'] = 1.0 - mean_hit_rate
            result[f'{zone_name}_mean_cell_hit_rate'] = mean_hit_rate
        result['frame_count'] = float(self._frame_count)
        return result

    def shutdown(self) -> None:
        for counts in self._zone_hit_counts.values():
            counts[:] = 0
        self._frame_count = 0
