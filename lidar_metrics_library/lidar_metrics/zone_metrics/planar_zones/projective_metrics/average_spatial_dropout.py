from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class AverageSpatialDropout(MetricsBase):
    """Per-frame spatial dropout averaged across all frames, in ray space.

    For each zone, the planar y-z face is divided into a uniform grid of physical
    cells (cell_size_m on the target surface), and every cell becomes its own mini
    cone: its four corners are projected through the lidar origin into (azimuth,
    elevation) and the cone is the min/max of those corner bearings. Each frame the
    point cloud is separated into those cones — a point is credited to the cell
    whose cone its ray falls inside, not by its raw Cartesian y-z position — and a
    fresh per-frame hit grid records which cones received at least one return that
    frame. The metric reports the mean miss rate across cells, which equals the
    expected fraction of cones missed by any single scan.

    Unlike SpatialDropout (which only flags permanently dead cones), this captures
    scan-pattern jitter: cones hit some frames and missed others contribute
    proportionally to the dropout. Working in ray space decouples dropout from
    range, reflecting the sensor's angular scan pattern. This metric consumes the
    projective (frustum-filtered) cloud, which already has the zone's y/z padding
    applied upstream; the padding is reported for reference but not re-applied.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self._zone_hit_counts: dict[str, np.ndarray] = {}
        self._zone_grid_meta: dict[str, dict] = {}
        self._frame_count: int = 0
        self._cell_size_m: float = 0.0
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        self._cell_size_m = float(self.config['lidar_metrics_parameters']['spatial_dropout']['cell_size_m'])
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

            self._zone_hit_counts[zb.name] = np.zeros((n_y, n_z), dtype=np.int64)
            self._zone_grid_meta[zb.name] = {
                'n_y': n_y,
                'n_z': n_z,
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
        padding. Dropout is averaged only over these so the padding ring — which
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

        # Count this frame even if a zone has zero points — that frame contributes 100% dropout for that zone.
        self._frame_count += 1

        for zb in self.profiles.zone_bounds:
            counts = self._zone_hit_counts.get(zb.name)
            if counts is None:
                continue
            meta = self._zone_grid_meta[zb.name]
            n_y = meta['n_y']
            n_z = meta['n_z']
            frame_hits = np.zeros((n_y, n_z), dtype=bool)

            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is not None and len(zone_pts) > 0:
                zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
                if len(zone_pts) > 0:
                    lx, ly, lz = meta['lidar']

                    # Each point's actual ray bearing from the lidar origin.
                    dx = zone_pts[:, 0] - lx
                    dy = zone_pts[:, 1] - ly
                    dz = zone_pts[:, 2] - lz
                    az = np.arctan2(dy, dx)
                    el = np.arctan2(dz, np.sqrt(dx * dx + dy * dy))

                    # Separate points into cones: column from the azimuth partition,
                    # then the cone(s) in that column whose elevation window contains
                    # the ray. A cone hit by multiple points still counts once.
                    iy = np.searchsorted(meta['az_edges'], az, side='right') - 1
                    in_az = (iy >= 0) & (iy < n_y)
                    sel = np.nonzero(in_az)[0]
                    if sel.size > 0:
                        iy_sel = iy[sel]
                        el_sel = el[sel][:, None]
                        el_lo = meta['el_min'][iy_sel]  # (M, n_z)
                        el_hi = meta['el_max'][iy_sel]  # (M, n_z)
                        hits = (el_sel >= el_lo) & (el_sel <= el_hi)  # (M, n_z)
                        pt_idx, iz = np.nonzero(hits)
                        frame_hits[iy_sel[pt_idx], iz] = True

            counts += frame_hits.astype(np.int64)

    def compute(self) -> dict[str, float]:
        result = {}
        for zone_name, counts in self._zone_hit_counts.items():
            meta = self._zone_grid_meta[zone_name]
            result[f'{zone_name}_y_padding_m'] = meta['y_padding']
            result[f'{zone_name}_z_padding_m'] = meta['z_padding']

            valid_cell = meta['valid_cell']
            total_cells = int(valid_cell.sum())
            if 0 == self._frame_count or 0 == total_cells:
                result[f'{zone_name}_avg_dropout_frac'] = 0.0
                result[f'{zone_name}_mean_cell_hit_rate'] = 0.0
                continue
            mean_hit_rate = float(counts[valid_cell].sum()) / float(self._frame_count * total_cells)
            result[f'{zone_name}_avg_dropout_frac'] = 1.0 - mean_hit_rate
            result[f'{zone_name}_mean_cell_hit_rate'] = mean_hit_rate
        result['frame_count'] = float(self._frame_count)
        return result

    def shutdown(self) -> None:
        for counts in self._zone_hit_counts.values():
            counts[:] = 0
        self._frame_count = 0
