# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

from __future__ import annotations

from dataclasses import dataclass, field, replace

import numpy as np

# Zone data structs + per-geometry bounds builders live in zones_utilities.
# FramePose / ZoneBounds re-exported here so existing importers keep working.
from lidar_transforms.tools.zones_utilities import (  # noqa: F401  (FramePose/ZoneBounds re-exported)
    FramePose,
    PlanarZoneBounds,
    PlanarZoneType,
    ROIConfig,
    ZONE_BOUNDS_BUILDERS,
    ZoneBounds,
)


# ---------------------------------------------------------------------------
# Profile-level data structs (not zone-geometry specific)
# ---------------------------------------------------------------------------

@dataclass
class NoiseRegion:
    """A spatial region with a specific noise model."""

    name: str
    center: np.ndarray    # shape (3,): [x, y, z] in map frame
    radius: float
    expected_sigma_m: float
    noise_type: str       # 'surface' | 'edge' | 'corner'
    z_min: float = 0.0
    z_max: float = 0.0


@dataclass
class FrustrumFilter:
    """Angular bounds for a projective frustum filter corresponding to a single zone."""

    name: str
    min_azimuth: float
    max_azimuth: float
    min_elevation: float
    max_elevation: float


@dataclass
class BaselineProfiles:
    """Complete set of spatial profiles derived from the ROI config and TF poses."""

    zone_bounds: list[ZoneBounds] = field(default_factory=list)
    lidar_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    frustrum_filter: list[FrustrumFilter] = field(default_factory=list)


# ---------------------------------------------------------------------------
# ProfileBuilder
# ---------------------------------------------------------------------------

class ProfileBuilder:
    """Builds BaselineProfiles from a ROIConfig and a set of resolved TF poses."""

    # Noise constants
    _EDGE_NOISE_RADIUS_M: float = 0.05
    _EDGE_NOISE_SIGMA_M: float = 0.015
    _CORNER_NOISE_RADIUS_M: float = 0.04
    _CORNER_NOISE_SIGMA_M: float = 0.025

    def build(
        self,
        roi_config: ROIConfig,
        frame_poses: dict[str, FramePose],
        rslidar_frame: str = 'default',
    ) -> BaselineProfiles:
        """Build complete baseline profiles.

        Args:
            roi_config: Validated ROI configuration.
            frame_poses: Mapping from TF frame name to its pose in the map frame.
                         Must contain poses for every zone frame and rslidar.
            rslidar_frame: Name of the LiDAR TF frame.

        Returns:
            Fully populated BaselineProfiles.

        Raises:
            KeyError: If a required frame is missing, or no bounds builder is
                registered for a zone's geometry.
        """
        lidar_pose = frame_poses[rslidar_frame]
        lidar_pos = lidar_pose.position

        # ------------------------------------------------------------------
        # 1. Resolve per-zone bounds via the geometry registry
        # ------------------------------------------------------------------
        zone_bounds: list[ZoneBounds] = []
        deferred_zones: list[int] = []  # planar zones (no explicit width) needing implicit y-bounds

        for zone_cfg in roi_config.zones:
            pose = frame_poses[zone_cfg.frame]
            builder = ZONE_BOUNDS_BUILDERS.resolve(type(zone_cfg.zone_type))
            zone_bounds.append(builder(zone_cfg, pose, lidar_pos))

            zt = zone_cfg.zone_type
            if isinstance(zt, PlanarZoneType) and zt.width is None:
                deferred_zones.append(len(zone_bounds) - 1)

        # Fill in implicit planar y-bounds now that every explicit zone is resolved.
        self._resolve_deferred_planar_bounds(zone_bounds, deferred_zones, frame_poses)

        # ------------------------------------------------------------------
        # 2. Build noise regions (planar seams only)
        # ------------------------------------------------------------------

        # not using the below yet

        planar_bounds = [zb for zb in zone_bounds if isinstance(zb, PlanarZoneBounds)]
        # noise_regions = self._build_noise_regions(planar_bounds)

        # ------------------------------------------------------------------
        # 3. Build frustrum filters (per-zone angular bounds, planar only)
        # ------------------------------------------------------------------
        frustrum_filters = self._build_frustrum_filters(planar_bounds, lidar_pos)

        return BaselineProfiles(
            zone_bounds=zone_bounds,
            lidar_position=lidar_pos.copy(),
            frustrum_filter=frustrum_filters,
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

    def _build_frustrum_filters(
        self, zone_bounds: list[PlanarZoneBounds], lidar_pos: np.ndarray
    ) -> list[FrustrumFilter]:
        """Project each planar zone's 4 corners into (az, el) relative to the lidar.

        The resulting window is axis-aligned in angle space, which slightly
        over-approximates the true angular footprint when the planar quad is
        tilted relative to the lidar. Assumes all zones lie in front of the
        lidar (forward = +x) so azimuth does not wrap across ±pi.
        """
        filters: list[FrustrumFilter] = []
        for zb in zone_bounds:
            corners = [
                (zb.x_surface, zb.y_min, zb.z_min),
                (zb.x_surface, zb.y_min, zb.z_max),
                (zb.x_surface, zb.y_max, zb.z_min),
                (zb.x_surface, zb.y_max, zb.z_max),
            ]
            azimuths: list[float] = []
            elevations: list[float] = []
            for cx, cy, cz in corners:
                dx = cx - lidar_pos[0]
                dy = cy - lidar_pos[1]
                dz = cz - lidar_pos[2]
                azimuths.append(float(np.arctan2(dy, dx)))
                elevations.append(float(np.arctan2(dz, np.sqrt(dx ** 2 + dy ** 2))))
            filters.append(FrustrumFilter(
                name=zb.name,
                min_azimuth=min(azimuths),
                max_azimuth=max(azimuths),
                min_elevation=min(elevations),
                max_elevation=max(elevations),
            ))
        return filters


    # probably dont need the below code


    # def _build_noise_regions(self, zone_bounds: list[PlanarZoneBounds]) -> list[NoiseRegion]:
    #     """Build edge noise regions at the y-boundary seam between adjacent zones."""
    #     noise_regions: list[NoiseRegion] = []

    #     sorted_zones = sorted(zone_bounds, key=lambda zb: zb.y_min)
    #     for i in range(len(sorted_zones) - 1):
    #         left_zone = sorted_zones[i]
    #         right_zone = sorted_zones[i + 1]
    #         boundary_y = (left_zone.y_max + right_zone.y_min) / 2.0
    #         boundary_x = (left_zone.x_surface + right_zone.x_surface) / 2.0
    #         z_min = min(left_zone.z_min, right_zone.z_min)
    #         z_max = max(left_zone.z_max, right_zone.z_max)
    #         z_center = (z_min + z_max) / 2.0
    #         noise_regions.append(NoiseRegion(
    #             name=f'edge_{left_zone.name}_{right_zone.name}',
    #             center=np.array([boundary_x, boundary_y, z_center]),
    #             radius=self._EDGE_NOISE_RADIUS_M,
    #             expected_sigma_m=self._EDGE_NOISE_SIGMA_M,
    #             noise_type='edge',
    #             z_min=z_min,
    #             z_max=z_max,
    #         ))

    #     return noise_regions
