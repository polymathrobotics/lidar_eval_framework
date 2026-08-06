from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class SpatialDropout(MetricsBase):
    """Detects spatial dead zones in the sensor's scan pattern per zone, in ray space.

    Each zone's planar y-z face is divided into a uniform grid of physical cells
    (cell_size_m on the target surface). Every cell becomes its own mini cone: its
    four corners are projected through the lidar origin into (azimuth, elevation),
    and the cell's cone is the min/max of those corner bearings. The point cloud is
    then separated into those cones — each point is assigned to the cell whose cone
    its ray falls inside, rather than by its raw Cartesian y-z position. A cell
    whose cone never receives a return across the whole run is a dead zone.

    Working in ray space (instead of binning raw y-z) decouples dropout from
    range: a point that lands slightly off the surface plane is still credited to
    the correct cell by its bearing, so the metric reflects the sensor's angular
    scan pattern — which is what beam/channel dropout actually is. This metric
    consumes the projective (frustum-filtered) cloud, which already has the zone's
    y/z padding applied upstream; the padding is reported in the results for
    reference but not re-applied here.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self._zone_hit_grids: dict[str, np.ndarray] = {}
        self._zone_grid_meta: dict[str, dict] = {}
        self._cell_size_m: float = 0.0
        self._max_dead_cells_reported: int = 0
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        params = self.config['lidar_metrics_parameters']['spatial_dropout']
        self._cell_size_m = float(params['cell_size_m'])
        self._max_dead_cells_reported = int(params.get('max_dead_cells_reported', 0))
        if self.profiles is None:
            return

        lx, ly, lz = (float(c) for c in self.profiles.lidar_position[:3])

        for zb in self.profiles.zone_bounds:
            n_y = max(1, int(np.ceil((zb.y_max - zb.y_min) / self._cell_size_m)))
            n_z = max(1, int(np.ceil((zb.z_max - zb.z_min) / self._cell_size_m)))

            # Physical cell-corner grid on the surface, projected into bearings to
            # form one cone per cell.
            y_edges = zb.y_min + np.arange(n_y + 1) * self._cell_size_m
            z_edges = zb.z_min + np.arange(n_z + 1) * self._cell_size_m
            depth = float(zb.x_surface) - lx

            # Azimuth depends only on y, so the n_y+1 y-edges give the cones'
            # ascending azimuth bounds (shared edges → columns partition cleanly).
            dy_edges = y_edges - ly
            az_edges = np.arctan2(dy_edges, depth)

            # Elevation at each of the (n_y+1)x(n_z+1) corners; a cone's elevation
            # bound is the min/max over its four corners.
            r_edges = np.sqrt(depth * depth + dy_edges * dy_edges)  # horiz range per y-edge
            el_corners = np.arctan2((z_edges - lz)[None, :], r_edges[:, None])  # (n_y+1, n_z+1)
            c00 = el_corners[:-1, :-1]
            c10 = el_corners[1:, :-1]
            c01 = el_corners[:-1, 1:]
            c11 = el_corners[1:, 1:]
            el_min = np.minimum(np.minimum(c00, c10), np.minimum(c01, c11))  # (n_y, n_z)
            el_max = np.maximum(np.maximum(c00, c10), np.maximum(c01, c11))  # (n_y, n_z)

            self._zone_hit_grids[zb.name] = np.zeros((n_y, n_z), dtype=bool)
            self._zone_grid_meta[zb.name] = {
                'n_y': n_y,
                'n_z': n_z,
                'y_min': zb.y_min,
                'z_min': zb.z_min,
                'y_padding': float(zb.y_padding),
                'z_padding': float(zb.z_padding),
                'az_edges': az_edges,
                'el_min': el_min,
                'el_max': el_max,
                'valid_cell': self._padded_cell_mask(zb, n_y, n_z),
                'lidar': (lx, ly, lz),
            }

    def _padded_cell_mask(self, zb, n_y: int, n_z: int) -> np.ndarray:
        """Cells whose centers fall inside the zone bounds shrunk by the y/z
        padding. Dropout is scored only over these so the padding ring — which
        the upstream projective filter intentionally excludes — isn't counted as
        dead and doesn't inflate the dropout rate."""
        y_centers = zb.y_min + (np.arange(n_y) + 0.5) * self._cell_size_m
        z_centers = zb.z_min + (np.arange(n_z) + 0.5) * self._cell_size_m
        y_ok = (y_centers >= zb.y_min + float(zb.y_padding)) & (y_centers <= zb.y_max - float(zb.y_padding))
        z_ok = (z_centers >= zb.z_min + float(zb.z_padding)) & (z_centers <= zb.z_max - float(zb.z_padding))
        return y_ok[:, None] & z_ok[None, :]

    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return

        self.pointcloud_by_zone = pointcloud_by_zone

        for zb in self.profiles.zone_bounds:
            grid = self._zone_hit_grids.get(zb.name)
            if grid is None:
                continue
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                continue
            zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
            if len(zone_pts) == 0:
                continue

            meta = self._zone_grid_meta[zb.name]
            lx, ly, lz = meta['lidar']
            n_y = meta['n_y']

            # Each point's actual ray bearing from the lidar origin.
            dx = zone_pts[:, 0] - lx
            dy = zone_pts[:, 1] - ly
            dz = zone_pts[:, 2] - lz
            az = np.arctan2(dy, dx)
            el = np.arctan2(dz, np.sqrt(dx * dx + dy * dy))

            # Separate points into cones: column from the azimuth partition, then
            # the cone(s) in that column whose elevation window contains the ray.
            iy = np.searchsorted(meta['az_edges'], az, side='right') - 1
            in_az = (iy >= 0) & (iy < n_y)
            sel = np.nonzero(in_az)[0]
            if sel.size == 0:
                continue
            iy_sel = iy[sel]
            el_sel = el[sel][:, None]

            el_lo = meta['el_min'][iy_sel]  # (M, n_z)
            el_hi = meta['el_max'][iy_sel]  # (M, n_z)
            hits = (el_sel >= el_lo) & (el_sel <= el_hi)  # (M, n_z)
            pt_idx, iz = np.nonzero(hits)
            grid[iy_sel[pt_idx], iz] = True

    def compute(self) -> dict[str, float]:
        result = {}
        for zone_name, grid in self._zone_hit_grids.items():
            meta = self._zone_grid_meta[zone_name]
            valid_cell = meta['valid_cell']

            total_cells = int(valid_cell.sum())
            dead_mask = valid_cell & ~grid
            dropout_cells = int(dead_mask.sum())

            result[f'{zone_name}_dropout_frac'] = float(dropout_cells / total_cells) if total_cells > 0 else 0.0
            result[f'{zone_name}_dropout_cell_count'] = float(dropout_cells)
            result[f'{zone_name}_total_cell_count'] = float(total_cells)
            result[f'{zone_name}_y_padding_m'] = meta['y_padding']
            result[f'{zone_name}_z_padding_m'] = meta['z_padding']

            y_min = meta['y_min']
            z_min = meta['z_min']
            dead_positions = np.argwhere(dead_mask)
            if self._max_dead_cells_reported > 0:
                dead_positions = dead_positions[:self._max_dead_cells_reported]
            for idx, (iy, iz) in enumerate(dead_positions):
                result[f'{zone_name}_dead_cell_{idx}_y_m'] = float(y_min + (iy + 0.5) * self._cell_size_m)
                result[f'{zone_name}_dead_cell_{idx}_z_m'] = float(z_min + (iz + 0.5) * self._cell_size_m)
        result['dead_cell_size_m'] = self._cell_size_m
        return result

    def shutdown(self) -> None:
        for grid in self._zone_hit_grids.values():
            grid[:] = False
