from lidar_metrics.metric_params_overrides.override_interface.base import OverrideInterfaceBase


class SpatialDropoutCellSizeMOverride(OverrideInterfaceBase):
    """Override for spatial_dropout.cell_size_m.

    Sizes the dropout grid from sensor-to-zone range: cell size grows linearly
    with distance along a line through the origin, pinned so that at
    ANCHOR_DISTANCE_M it equals ANCHOR_CELL_SIZE_M (today's static default):

        cell_size = (ANCHOR_CELL_SIZE_M / ANCHOR_DISTANCE_M) * distance

    Distance is floored at MIN_DISTANCE_M (physical tests are never closer than
    this), which bounds the finest cell / max cell count. The result is capped at
    the zone's smaller dimension (the largest square that fits inside the zone),
    so a cell is never larger than the zone itself. Computed per planar zone, then
    averaged into the single config value.
    """

    ANCHOR_DISTANCE_M = 3.8    # at this range, cell size == ANCHOR_CELL_SIZE_M
    ANCHOR_CELL_SIZE_M = 0.05
    MIN_DISTANCE_M = 1.0       # tests never closer than 1 m; clamps the finest cell

    def retrieve_param(self) -> float:
        cell_sizes = [self._cell_size_for_zone(zb) for zb in self._planar_zones()]
        if not cell_sizes:
            return self.ANCHOR_CELL_SIZE_M  # no planar zones to size from
        return float(sum(cell_sizes) / len(cell_sizes))

    def _planar_zones(self) -> list:
        zones = getattr(self.profiles, "zone_bounds", []) if self.profiles is not None else []
        # Both planar and cylindrical bounds carry x_surface; only planar bounds
        # carry y_min/y_max, so discriminate on those.
        return [zb for zb in zones if hasattr(zb, "y_max")]

    def _cell_size_for_zone(self, zb) -> float:
        # 1D distance from the lidar to this zone, along x (the zone surface is the
        # plane x = x_surface). lidar_position is xyz in the map frame.
        distance = abs(zb.x_surface - float(self.profiles.lidar_position[0]))
        distance = max(distance, self.MIN_DISTANCE_M)

        # Line through the origin, pinned at (ANCHOR_DISTANCE_M, ANCHOR_CELL_SIZE_M).
        cell_size = (self.ANCHOR_CELL_SIZE_M / self.ANCHOR_DISTANCE_M) * distance

        # Cap at the largest square that still fits inside the zone (the smaller
        # dimension), so a cell is never bigger than the zone itself.
        max_cell_size = min(zb.y_max - zb.y_min, zb.z_max - zb.z_min)
        return float(min(cell_size, max_cell_size))
