from __future__ import annotations

from dataclasses import replace

# The ZoneEngine (in zones_api) routes each zone to its geometry plugin, so
# ProfileBuilder never imports a concrete geometry to *build* bounds.
# FramePose / ZoneBounds re-exported here so existing importers keep working.
from lidar_zones.zones_api import ZoneEngine
from lidar_zones.zones_api.profile_types import (  # noqa: F401  (FramePose/ZoneBounds re-exported)
    BaselineProfiles,
    FramePose,
    ROIConfig,
    ZoneBounds,
)
from lidar_zones.zones_api.zone_plugins.planar import PlanarZonePlugin

# Planar's structs are nested in its plugin; alias them to local names for the
# planar-only deferred-width resolution below (planar zones may omit an explicit
# width, so their y-bounds are inferred from neighbors — a cross-zone step no
# single-zone plugin can own).
PlanarZoneType = PlanarZonePlugin.PlanarZoneType
PlanarZoneBounds = PlanarZonePlugin.PlanarZoneBounds


# ---------------------------------------------------------------------------
# ProfileBuilder
# ---------------------------------------------------------------------------

class ProfileBuilder:
    """Builds BaselineProfiles from a ROIConfig and a set of resolved TF poses."""

    def __init__(self) -> None:
        # Builds each zone's bounds by routing its ZoneType to the geometry plugin.
        self._engine = ZoneEngine()

    def build(
        self,
        roi_config: ROIConfig,
        frame_poses: dict[str, FramePose],
        lidar_frame: str = 'default',
    ) -> BaselineProfiles:
        """Build complete baseline profiles.

        Args:
            roi_config: Validated ROI configuration.
            frame_poses: Mapping from TF frame name to its pose in the map frame.
                         Must contain poses for every zone frame and the lidar.
            lidar_frame: Name of the LiDAR TF frame.

        Returns:
            Fully populated BaselineProfiles.

        Raises:
            KeyError: If a required frame is missing, or no plugin is registered
                for a zone's geometry.
        """
        lidar_pose = frame_poses[lidar_frame]
        lidar_pos = lidar_pose.position

        # Resolve each zone's bounds through the engine (which routes to the
        # geometry plugin). Planar zones without an explicit width defer their
        # y-bounds to a second pass once every explicit zone is resolved.
        zone_bounds: list[ZoneBounds] = []
        deferred_zones: list[int] = []  # planar zones (no explicit width) needing implicit y-bounds

        for zone_cfg in roi_config.zones:
            pose = frame_poses[zone_cfg.frame]
            zone_bounds.append(self._engine.build(zone_cfg, pose, lidar_pos).bounds)

            zt = zone_cfg.zone_type
            if isinstance(zt, PlanarZoneType) and zt.width is None:
                deferred_zones.append(len(zone_bounds) - 1)

        # Fill in implicit planar y-bounds now that every explicit zone is resolved.
        self._resolve_deferred_planar_bounds(zone_bounds, deferred_zones, frame_poses)

        return BaselineProfiles(
            zone_bounds=zone_bounds,
            lidar_position=lidar_pos.copy(),
        )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _resolve_deferred_planar_bounds(
        self,
        zone_bounds: list[ZoneBounds],
        deferred_zones: list[int],
        frame_poses: dict[str, FramePose],
    ) -> None:
        """Fill implicit y-bounds for planar zones that had no explicit width.

        Each deferred zone takes a symmetric y-window sized from its nearest
        explicit-zone y-edge. Mutates `zone_bounds` in place.
        """
        if not deferred_zones:
            return

        deferred_set = set(deferred_zones)
        explicit_y_values = [
            y_val
            for i, zb in enumerate(zone_bounds)
            if i not in deferred_set and isinstance(zb, PlanarZoneBounds)
            for y_val in (zb.y_min, zb.y_max)
        ]

        for idx in deferred_zones:
            zb = zone_bounds[idx]
            center_y = frame_poses[zb.zone_config.frame].position[1]
            y_min, y_max = self._resolve_implicit_bounds(center_y, explicit_y_values)
            zone_bounds[idx] = replace(zb, y_min=y_min, y_max=y_max)

    def _resolve_implicit_bounds(
        self, center_y: float, explicit_y_values: list[float]
    ) -> tuple[float, float]:
        """Resolve y-bounds for a planar zone without an explicit width."""
        if not explicit_y_values:
            return (center_y - 2.0, center_y + 2.0)

        nearest = min(explicit_y_values, key=lambda v: abs(v - center_y))
        half_width = abs(nearest - center_y)
        if half_width < 1e-9:
            half_width = 2.0
        return (center_y - half_width, center_y + half_width)
