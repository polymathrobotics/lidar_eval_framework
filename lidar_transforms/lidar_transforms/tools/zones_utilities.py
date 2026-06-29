# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

"""Single source of truth for zone geometry.

Holds every per-geometry zone data struct, the registry maps that route a zone
to its geometry-specific handler, and the skinny functions those maps point to.
To add a new zone geometry you add: its dataclasses here, the skinny handler for
each registry here (or, for ROS-dependent handlers like markers, register from
the owning module), and nothing else — the consuming modules dispatch through
the registries and need no edits.

This module is intentionally ROS-free (numpy only) so it stays importable
standalone. ROS-dependent handlers (e.g. RViz markers) register their bodies
into the registries defined here from their own modules.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Depth half-thickness (meters) of the planar spatial slab around the surface.
ROI_X_PADDING_M: float = 0.1

# Maps a zone's geometry (the YAML `type` field) to the surface-noise model
# used to derive ZoneConfig.noise_sigma_m. A cylinder presents a curved surface.
GEOMETRY_SURFACE: dict[str, str] = {
    'planar': 'planar',
    'cylindrical': 'curved',
}


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

class ZoneRegistry:


    def __init__(self, name: str) -> None:
        self._name = name
        self._handlers: dict = {}

    def register(self, key) -> Callable:
        """Decorator: register a handler under `key` (a type or a string)."""
        def _decorator(fn: Callable) -> Callable:
            self._handlers[key] = fn
            return fn
        return _decorator

    def resolve(self, key):
        """Return the handler for `key`, raising a clear error if missing."""
        if key not in self._handlers:
            label = getattr(key, '__name__', repr(key))
            raise KeyError(f'{self._name}: no handler registered for {label}')
        return self._handlers[key]

    def for_obj(self, obj):
        """Return the handler registered for `type(obj)`."""
        return self.resolve(type(obj))


# One registry per dispatch domain. The skinny handlers below (and markers in
# marker_builder) populate these.
ZONE_TYPE_PARSERS = ZoneRegistry('zone_type_parsers')          # geometry str  -> ZoneType
ZONE_BOUNDS_BUILDERS = ZoneRegistry('zone_bounds_builders')    # ZoneType cls  -> ZoneBounds
SPATIAL_MASKS = ZoneRegistry('spatial_masks')                  # ZoneBounds cls -> bool mask
PROJECTIVE_MASKS = ZoneRegistry('projective_masks')            # ZoneBounds cls -> bool mask
ZONE_TYPE_TO_DICT = ZoneRegistry('zone_type_to_dict')          # ZoneType cls  -> dict
ZONE_TYPE_FROM_DICT = ZoneRegistry('zone_type_from_dict')      # geometry str  -> ZoneType
ZONE_BOUNDS_TO_DICT = ZoneRegistry('zone_bounds_to_dict')      # ZoneBounds cls -> dict
ZONE_BOUNDS_FROM_DICT = ZoneRegistry('zone_bounds_from_dict')  # geometry str  -> ZoneBounds
EXPECTED_ZONE_FIELDS = ZoneRegistry('expected_zone_fields')    # ZoneBounds cls -> ExpectedZone field dict (lidar-relative)
MARKER_BUILDERS = ZoneRegistry('marker_builders')              # ZoneBounds cls -> int (populated by marker_builder)


# ---------------------------------------------------------------------------
# Pose / config data structs
# ---------------------------------------------------------------------------

@dataclass
class FramePose:
    """Pose of a TF frame expressed in the map frame."""

    position: np.ndarray   # shape (3,): [x, y, z]
    rotation: np.ndarray   # shape (3, 3): rotation matrix


@dataclass
class PlanarZoneType:
    """Geometry parameters specific to a planar (flat-surface) ROI zone.

    `width` is optional — when omitted the y-bounds are resolved implicitly
    from neighboring zones by the ProfileBuilder.
    """

    z_bounds: tuple[float, float]
    width: Optional[float] = None
    # Projective frustum padding (meters). Applied INWARD per side by the
    # ROI filter to shrink the angular cone; metrics that compute geometry-
    # based expected counts must subtract 2× these values from w/h so their
    # denominator matches the actual filtered footprint.
    y_padding: float = 0.0
    z_padding: float = 0.0


@dataclass
class CylindricalZoneType:
    """Geometry parameters specific to a cylindrical ROI zone."""

    height: float
    radius: float
    # radius_padding/height_padding (meters): projective filter applies INWARD
    # (shrinking the surface); spatial filter applies OUTWARD (radius_padding→Y,
    # height_padding→Z). outward_radius_padding is spatial-only, growing X (fwd/back).
    radius_padding: float = 0.0
    height_padding: float = 0.0
    outward_radius_padding: float = 0.0


ZoneType = CylindricalZoneType | PlanarZoneType


@dataclass
class ZoneConfig:
    name: str
    frame: str
    color: str
    expected_intensity: float
    noise_sigma_m: float
    zone_type: ZoneType


@dataclass
class ROIConfig:
    zones: list[ZoneConfig] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Resolved-bounds data structs
# ---------------------------------------------------------------------------

@dataclass
class PlanarZoneBounds:
    """Resolved spatial bounds for a planar ROI zone."""

    name: str
    zone_config: ZoneConfig
    y_min: float
    y_max: float
    x_surface: float         # absolute X of surface in map frame
    expected_depth_m: float  # distance from rslidar to surface along X
    z_min: float = 0.0       # per-zone z lower bound
    z_max: float = 0.0       # per-zone z upper bound
    x_min: float = 0.0       # x_surface - ROI_X_PADDING_M
    x_max: float = 0.0       # x_surface + ROI_X_PADDING_M
    # Projective frustum padding (meters), applied INWARD per side by the
    # ROI filter. Metrics computing geometry-based expected point counts on
    # the projective cloud must subtract 2× these values from w/h.
    y_padding: float = 0.0
    z_padding: float = 0.0


@dataclass
class CylindricalZoneBounds:
    """Resolved spatial bounds for a cylindrical ROI zone (axis vertical, +Z).

    z_min/z_max/radius are the FULL spatial extent. The paddings are dual-purpose:
    the projective filter applies radius_padding/height_padding INWARD (shrinking
    the surface), while the spatial filter applies all three OUTWARD (growing a
    bounding prism), one padding per axis.
    """

    name: str
    zone_config: ZoneConfig
    center_x: float          # cylinder axis X in map frame
    center_y: float          # cylinder axis Y in map frame
    radius: float            # full cylinder radius (no padding)
    x_surface: float         # nearest face X toward the lidar (center_x - radius)
    expected_depth_m: float  # distance from rslidar to the nearest face along X
    z_min: float = 0.0       # full per-zone z lower bound
    z_max: float = 0.0       # full per-zone z upper bound
    # Dual-use paddings (meters): projective filter applies these INWARD; spatial
    # filter applies them OUTWARD — radius_padding grows Y (sides), height_padding grows Z.
    radius_padding: float = 0.0
    height_padding: float = 0.0
    # Spatial-only padding (meters), applied OUTWARD along X (lidar view axis, fwd/back).
    outward_radius_padding: float = 0.0


ZoneBounds = PlanarZoneBounds | CylindricalZoneBounds


# ---------------------------------------------------------------------------
# Skinny handlers: parse YAML geometry fields -> ZoneType
# ---------------------------------------------------------------------------

@ZONE_TYPE_PARSERS.register('planar')
def parse_planar_zone_type(raw: dict, location: str) -> PlanarZoneType:
    """Parse and validate the planar-specific fields (z_bounds / width / paddings)."""
    z_bounds_raw = raw.get('z_bounds')
    if not isinstance(z_bounds_raw, (list, tuple)) or len(z_bounds_raw) != 2:
        raise ValueError(f'"z_bounds" in {location} must be a list of two numbers')

    z_min, z_max = float(z_bounds_raw[0]), float(z_bounds_raw[1])
    if z_min >= z_max:
        raise ValueError(
            f'"z_bounds" min ({z_min}) must be less than max ({z_max}) in {location}'
        )

    width_raw = raw.get('width')
    width: Optional[float] = None
    if width_raw is not None:
        width = float(width_raw)
        if width <= 0.0:
            raise ValueError(f'"width" must be > 0 in {location}, got {width}')

    y_padding = float(raw.get('y_padding', 0.0))
    z_padding = float(raw.get('z_padding', 0.0))
    if y_padding < 0.0 or z_padding < 0.0:
        raise ValueError(f'"y_padding"/"z_padding" must be >= 0 in {location}')

    return PlanarZoneType(z_bounds=(z_min, z_max), width=width, y_padding=y_padding, z_padding=z_padding)


@ZONE_TYPE_PARSERS.register('cylindrical')
def parse_cylindrical_zone_type(raw: dict, location: str) -> CylindricalZoneType:
    """Parse and validate the cylindrical-specific fields (height / radius / paddings)."""
    height_raw = raw.get('height')
    if height_raw is None:
        raise ValueError(f'Missing "height" field in {location}')
    height = float(height_raw)
    if height <= 0.0:
        raise ValueError(f'"height" must be > 0 in {location}, got {height}')

    radius_raw = raw.get('radius')
    if radius_raw is None:
        raise ValueError(f'Missing "radius" field in {location}')
    radius = float(radius_raw)
    if radius <= 0.0:
        raise ValueError(f'"radius" must be > 0 in {location}, got {radius}')

    radius_padding = float(raw.get('radius_padding', 0.0))
    height_padding = float(raw.get('height_padding', 0.0))
    if radius_padding < 0.0 or height_padding < 0.0:
        raise ValueError(f'"radius_padding"/"height_padding" must be >= 0 in {location}')
    if radius_padding >= radius:
        raise ValueError(
            f'"radius_padding" ({radius_padding}) must be < radius ({radius}) in {location}'
        )
    if 2.0 * height_padding >= height:
        raise ValueError(
            f'"height_padding" ({height_padding}) too large in {location}: '
            f'2x must be less than height ({height})'
        )

    # Spatial-only padding: grows the bounding prism OUTWARD along the lidar's view
    # axis (forward/back). Unbounded above — unlike the inward paddings it doesn't eat
    # into the zone, so it only has to be non-negative.
    outward_radius_padding = float(raw.get('outward_radius_padding', 0.0))
    if outward_radius_padding < 0.0:
        raise ValueError(f'"outward_radius_padding" must be >= 0 in {location}')

    return CylindricalZoneType(
        height=height, radius=radius, radius_padding=radius_padding,
        height_padding=height_padding, outward_radius_padding=outward_radius_padding,
    )


# ---------------------------------------------------------------------------
# Skinny handlers: build resolved ZoneBounds from ZoneConfig + TF pose
# ---------------------------------------------------------------------------

@ZONE_BOUNDS_BUILDERS.register(PlanarZoneType)
def build_planar_zone_bounds(zone_cfg: ZoneConfig, pose: FramePose, lidar_pos: np.ndarray) -> PlanarZoneBounds:
    """Resolve a planar zone's bounds. Zones without an explicit width get
    placeholder y-bounds here; ProfileBuilder fills them from neighbors."""
    zt: PlanarZoneType = zone_cfg.zone_type
    x_surface = pose.position[0]
    expected_depth_m = x_surface - lidar_pos[0]

    if zt.width is not None:
        half = zt.width / 2.0
        y_min, y_max = pose.position[1] - half, pose.position[1] + half
    else:
        y_min, y_max = 0.0, 0.0  # deferred — resolved from neighbors later

    return PlanarZoneBounds(
        name=zone_cfg.name,
        zone_config=zone_cfg,
        y_min=y_min,
        y_max=y_max,
        x_surface=x_surface,
        expected_depth_m=expected_depth_m,
        z_min=zt.z_bounds[0],
        z_max=zt.z_bounds[1],
        x_min=x_surface - ROI_X_PADDING_M,
        x_max=x_surface + ROI_X_PADDING_M,
        y_padding=zt.y_padding,
        z_padding=zt.z_padding,
    )


@ZONE_BOUNDS_BUILDERS.register(CylindricalZoneType)
def build_cylindrical_zone_bounds(
    zone_cfg: ZoneConfig, pose: FramePose, lidar_pos: np.ndarray
) -> CylindricalZoneBounds:
    """Resolve a cylindrical zone's FULL bounds from its radius/height + TF pose.

    The TF frame sits at the cylinder's centroid, so z spans pose.z ± height/2.
    radius and z are the full spatial extent; paddings are carried through for
    the projective filter to apply inward.
    """
    zt: CylindricalZoneType = zone_cfg.zone_type
    center_x = pose.position[0]
    center_y = pose.position[1]
    center_z = pose.position[2]

    half_height = zt.height / 2.0
    z_min = center_z - half_height
    z_max = center_z + half_height

    # Near face toward the lidar (assumed at x < zone, forward = +x).
    x_surface = center_x - zt.radius
    expected_depth_m = x_surface - lidar_pos[0]

    return CylindricalZoneBounds(
        name=zone_cfg.name,
        zone_config=zone_cfg,
        center_x=center_x,
        center_y=center_y,
        radius=zt.radius,
        x_surface=x_surface,
        expected_depth_m=expected_depth_m,
        z_min=z_min,
        z_max=z_max,
        radius_padding=zt.radius_padding,
        height_padding=zt.height_padding,
        outward_radius_padding=zt.outward_radius_padding,
    )


# ---------------------------------------------------------------------------
# Skinny handlers: spatial filter masks (FULL extent, no padding)
# ---------------------------------------------------------------------------

@SPATIAL_MASKS.register(PlanarZoneBounds)
def planar_spatial_mask(zb: PlanarZoneBounds, xyz_map: np.ndarray) -> np.ndarray:
    """Axis-aligned bounding-box mask over the full planar zone extent."""
    return (
        (xyz_map[:, 0] >= zb.x_min) & (xyz_map[:, 0] <= zb.x_max)
        & (xyz_map[:, 1] >= zb.y_min) & (xyz_map[:, 1] <= zb.y_max)
        & (xyz_map[:, 2] >= zb.z_min) & (xyz_map[:, 2] <= zb.z_max)
    )


@SPATIAL_MASKS.register(CylindricalZoneBounds)
def cylindrical_spatial_mask(zb: CylindricalZoneBounds, xyz_map: np.ndarray) -> np.ndarray:
    """Axis-aligned bounding prism enclosing the whole cylinder, grown OUTWARD.

    Unlike the projective filter (which pads the surface INWARD), the spatial filter
    keeps the cylinder's full shape and expands a box around it, one padding per axis:
      * X (lidar view axis, forward/back): radius + outward_radius_padding
      * Y (sides):                          radius + radius_padding
      * Z (height):                         z_min/z_max -/+ height_padding
    """
    x_min = zb.center_x - zb.radius - zb.outward_radius_padding
    x_max = zb.center_x + zb.radius + zb.outward_radius_padding
    y_min = zb.center_y - zb.radius - zb.radius_padding
    y_max = zb.center_y + zb.radius + zb.radius_padding
    z_min = zb.z_min - zb.height_padding
    z_max = zb.z_max + zb.height_padding
    return (
        (xyz_map[:, 0] >= x_min) & (xyz_map[:, 0] <= x_max)
        & (xyz_map[:, 1] >= y_min) & (xyz_map[:, 1] <= y_max)
        & (xyz_map[:, 2] >= z_min) & (xyz_map[:, 2] <= z_max)
    )


# ---------------------------------------------------------------------------
# Skinny handlers: projective filter masks (angular cone, padded INWARD)
# ---------------------------------------------------------------------------

@PROJECTIVE_MASKS.register(PlanarZoneBounds)
def planar_projective_mask(
    zb: PlanarZoneBounds,
    az: np.ndarray,
    el: np.ndarray,
    lidar_position: np.ndarray,
    y_padding: dict[str, float],
    z_padding: dict[str, float],
) -> np.ndarray:
    """Angular window from the planar zone's 4 corners, padded INWARD."""
    lx, ly, lz = lidar_position
    y_pad = float(y_padding.get(zb.name, 0.0))
    z_pad = float(z_padding.get(zb.name, 0.0))

    y_min_p = zb.y_min + y_pad
    y_max_p = zb.y_max - y_pad
    z_min_p = zb.z_min + z_pad
    z_max_p = zb.z_max - z_pad

    corners = [
        (zb.x_surface, y_min_p, z_min_p),
        (zb.x_surface, y_min_p, z_max_p),
        (zb.x_surface, y_max_p, z_min_p),
        (zb.x_surface, y_max_p, z_max_p),
    ]
    azimuths: list[float] = []
    elevations: list[float] = []
    for cx, cy, cz in corners:
        cdx = cx - lx
        cdy = cy - ly
        cdz = cz - lz
        azimuths.append(float(np.arctan2(cdy, cdx)))
        elevations.append(float(np.arctan2(cdz, np.sqrt(cdx ** 2 + cdy ** 2))))

    return (
        (az >= min(azimuths)) & (az <= max(azimuths))
        & (el >= min(elevations)) & (el <= max(elevations))
    )


@PROJECTIVE_MASKS.register(CylindricalZoneBounds)
def cylindrical_projective_mask(
    zb: CylindricalZoneBounds,
    az: np.ndarray,
    el: np.ndarray,
    lidar_position: np.ndarray,
    y_padding: dict[str, float],
    z_padding: dict[str, float],
) -> np.ndarray:
    """Angular cone from the cylinder silhouette, padded INWARD.

    Azimuth half-width = arcsin(r' / d) about the center azimuth, where
    r' = radius - radius_padding and d is the horizontal lidar→axis distance.
    Elevation spans the padded z-band measured at the near-face distance
    (d - r'). y_padding/z_padding are unused (the cylinder carries its own
    radius_padding/height_padding); they keep the registry signature uniform.
    """
    lx, ly, lz = lidar_position
    cdx = zb.center_x - lx
    cdy = zb.center_y - ly
    d = float(np.sqrt(cdx ** 2 + cdy ** 2))

    radius = max(zb.radius - zb.radius_padding, 0.0)
    z_min_p = zb.z_min + zb.height_padding
    z_max_p = zb.z_max - zb.height_padding

    center_az = float(np.arctan2(cdy, cdx))
    ratio = min(radius / d, 1.0) if d > 0.0 else 1.0
    half_az = float(np.arcsin(ratio))

    d_near = max(d - radius, 1e-6)
    el_min = float(np.arctan2(z_min_p - lz, d_near))
    el_max = float(np.arctan2(z_max_p - lz, d_near))

    return (
        (az >= center_az - half_az) & (az <= center_az + half_az)
        & (el >= el_min) & (el <= el_max)
    )


# ---------------------------------------------------------------------------
# Skinny handlers: (de)serialization of ZoneType
# ---------------------------------------------------------------------------

@ZONE_TYPE_TO_DICT.register(PlanarZoneType)
def planar_zone_type_to_dict(zt: PlanarZoneType) -> dict:
    return {
        'geometry': 'planar',
        'z_bounds': list(zt.z_bounds),
        'width': zt.width,
        'y_padding': zt.y_padding,
        'z_padding': zt.z_padding,
    }


@ZONE_TYPE_TO_DICT.register(CylindricalZoneType)
def cylindrical_zone_type_to_dict(zt: CylindricalZoneType) -> dict:
    return {
        'geometry': 'cylindrical',
        'height': zt.height,
        'radius': zt.radius,
        'radius_padding': zt.radius_padding,
        'height_padding': zt.height_padding,
    }


@ZONE_TYPE_FROM_DICT.register('planar')
def planar_zone_type_from_dict(d: dict) -> PlanarZoneType:
    return PlanarZoneType(
        z_bounds=(d['z_bounds'][0], d['z_bounds'][1]),
        width=d.get('width'),
        y_padding=float(d.get('y_padding', 0.0)),
        z_padding=float(d.get('z_padding', 0.0)),
    )


@ZONE_TYPE_FROM_DICT.register('cylindrical')
def cylindrical_zone_type_from_dict(d: dict) -> CylindricalZoneType:
    return CylindricalZoneType(
        height=d['height'],
        radius=d['radius'],
        radius_padding=float(d.get('radius_padding', 0.0)),
        height_padding=float(d.get('height_padding', 0.0)),
    )


# ---------------------------------------------------------------------------
# Skinny handlers: (de)serialization of ZoneBounds
# ---------------------------------------------------------------------------

@ZONE_BOUNDS_TO_DICT.register(PlanarZoneBounds)
def planar_bounds_to_dict(zb: PlanarZoneBounds) -> dict:
    return {
        'geometry': 'planar',
        'y_min': zb.y_min,
        'y_max': zb.y_max,
        'x_surface': zb.x_surface,
        'expected_depth_m': zb.expected_depth_m,
        'z_min': zb.z_min,
        'z_max': zb.z_max,
        'x_min': zb.x_min,
        'x_max': zb.x_max,
        'y_padding': zb.y_padding,
        'z_padding': zb.z_padding,
    }


@ZONE_BOUNDS_TO_DICT.register(CylindricalZoneBounds)
def cylindrical_bounds_to_dict(zb: CylindricalZoneBounds) -> dict:
    return {
        'geometry': 'cylindrical',
        'center_x': zb.center_x,
        'center_y': zb.center_y,
        'radius': zb.radius,
        'x_surface': zb.x_surface,
        'expected_depth_m': zb.expected_depth_m,
        'z_min': zb.z_min,
        'z_max': zb.z_max,
        'radius_padding': zb.radius_padding,
        'height_padding': zb.height_padding,
    }


@ZONE_BOUNDS_FROM_DICT.register('planar')
def planar_bounds_from_dict(d: dict, zone_config: ZoneConfig) -> PlanarZoneBounds:
    return PlanarZoneBounds(
        name=d['name'],
        zone_config=zone_config,
        y_min=d['y_min'],
        y_max=d['y_max'],
        x_surface=d['x_surface'],
        expected_depth_m=d['expected_depth_m'],
        z_min=d['z_min'],
        z_max=d['z_max'],
        x_min=d['x_min'],
        x_max=d['x_max'],
        y_padding=float(d.get('y_padding', 0.0)),
        z_padding=float(d.get('z_padding', 0.0)),
    )


@ZONE_BOUNDS_FROM_DICT.register('cylindrical')
def cylindrical_bounds_from_dict(d: dict, zone_config: ZoneConfig) -> CylindricalZoneBounds:
    return CylindricalZoneBounds(
        name=d['name'],
        zone_config=zone_config,
        center_x=d['center_x'],
        center_y=d['center_y'],
        radius=d['radius'],
        x_surface=d['x_surface'],
        expected_depth_m=d['expected_depth_m'],
        z_min=d['z_min'],
        z_max=d['z_max'],
        radius_padding=float(d.get('radius_padding', 0.0)),
        height_padding=float(d.get('height_padding', 0.0)),
    )


# ---------------------------------------------------------------------------
# Skinny handlers: ExpectedZone field dicts for visualization (lidar-relative).
# Each geometry returns ONLY the fields it needs; the node fills those onto the
# ExpectedZone msg and leaves the rest at defaults. Keys map 1:1 to msg fields;
# positions are shifted by the lidar position, lengths (radius) are left as-is.
# ---------------------------------------------------------------------------

@EXPECTED_ZONE_FIELDS.register(PlanarZoneBounds)
def planar_expected_fields(zb: PlanarZoneBounds, lidar_pos: np.ndarray) -> dict:
    """Flat plane at x with y/z extents — a box is exact for a plane."""
    lx, ly, lz = lidar_pos
    return {
        'geometry': 'planar',
        'x': zb.x_surface - lx,
        'y_min': zb.y_min - ly,
        'y_max': zb.y_max - ly,
        'z_min': zb.z_min - lz,
        'z_max': zb.z_max - lz,
    }


@EXPECTED_ZONE_FIELDS.register(CylindricalZoneBounds)
def cylindrical_expected_fields(zb: CylindricalZoneBounds, lidar_pos: np.ndarray) -> dict:
    """Half-cylindrical prism: true axis (center_x/center_y) + radius + z span.

    The near-facing semicircle is derived by the renderer from the axis relative
    to the lidar (which sits at the origin in this lidar-relative frame). Only
    the cylinder-relevant fields are returned; the planar box fields stay at
    their msg defaults.
    """
    lx, ly, lz = lidar_pos
    return {
        'geometry': 'cylindrical',
        'center_x': zb.center_x - lx,
        'center_y': zb.center_y - ly,
        'radius': zb.radius,
        'z_min': zb.z_min - lz,
        'z_max': zb.z_max - lz,
    }
