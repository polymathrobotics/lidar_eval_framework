"""Planar zone geometry plugin — everything about a flat-surface ROI zone.

The geometry's data structs (ZoneType + ZoneBounds) are nested inside the plugin
class, so a single class holds the geometry's data *and* behavior. The geometry
string is declared once (`geometry = 'planar'`) and referenced everywhere else
via `self.geometry` / `cls.geometry`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from urdf_parser_py.urdf import Box, Color, Link, LinkMaterial, Visual

from lidar_zones.zones_api.profile_types import FramePose, ZoneBounds, ZoneConfig, ZoneType
from lidar_zones.zones_api.zone_plugin_api import ZoneTypePlugin


# Depth half-thickness (meters) of the planar spatial slab around the surface.
ROI_X_PADDING_M: float = 0.1


class PlanarZonePlugin(ZoneTypePlugin):
    """Plugin for planar zones. `self.bounds` is a PlanarZoneBounds."""

    # ---- data structs (nested: the plugin owns its geometry's structs) -------

    @dataclass
    class PlanarZoneType(ZoneType):
        """Geometry parameters specific to a planar (flat-surface) ROI zone.

        `width` is optional — when omitted the y-bounds are resolved implicitly
        from neighboring zones by the ProfileBuilder.
        """

        z_bounds: tuple[float, float]
        width: Optional[float] = None
        y_padding: float = 0.0
        z_padding: float = 0.0

    @dataclass
    class PlanarZoneBounds(ZoneBounds):
        """Resolved spatial bounds for a planar ROI zone."""

        name: str
        zone_config: ZoneConfig
        y_min: float
        y_max: float
        x_surface: float         # absolute X of surface in map frame
        expected_depth_m: float  # distance from lidar to surface along X
        z_min: float = 0.0
        z_max: float = 0.0
        x_min: float = 0.0       # x_surface - ROI_X_PADDING_M
        x_max: float = 0.0       # x_surface + ROI_X_PADDING_M
        y_padding: float = 0.0   # projective inset per side (applied INWARD by the filter)
        z_padding: float = 0.0

    zone_type_cls = PlanarZoneType
    bounds_cls = PlanarZoneBounds

    # ---- construction --------------------------------------------------------

    @classmethod
    def parse_zone_type(cls, raw: dict, location: str) -> ZoneType:
        z_bounds_raw = raw.get('z_bounds')
        if not isinstance(z_bounds_raw, (list, tuple)) or len(z_bounds_raw) != 2:
            raise ValueError(f'"z_bounds" in {location} must be a list of two numbers')

        z_min, z_max = float(z_bounds_raw[0]), float(z_bounds_raw[1])
        if z_min >= z_max:
            raise ValueError(f'"z_bounds" min ({z_min}) must be less than max ({z_max}) in {location}')

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

        return cls.PlanarZoneType(z_bounds=(z_min, z_max), width=width, y_padding=y_padding, z_padding=z_padding)

    @classmethod
    def build(cls, zone_cfg: ZoneConfig, pose: FramePose, lidar_pos: np.ndarray) -> "PlanarZonePlugin":
        """Resolve a planar zone's bounds. Zones without an explicit width get
        placeholder y-bounds here; ProfileBuilder fills them from neighbors."""
        zt = zone_cfg.zone_type
        x_surface = pose.position[0]
        expected_depth_m = x_surface - lidar_pos[0]

        if zt.width is not None:
            half = zt.width / 2.0
            y_min, y_max = pose.position[1] - half, pose.position[1] + half
        else:
            y_min, y_max = 0.0, 0.0  # deferred — resolved from neighbors later

        bounds = cls.PlanarZoneBounds(
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
        return cls(bounds)

    @classmethod
    def from_dict(cls, d: dict, zone_config: ZoneConfig) -> "PlanarZonePlugin":
        bounds = cls.PlanarZoneBounds(
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
        return cls(bounds)

    @classmethod
    def zone_type_to_dict(cls, zone_type: ZoneType) -> dict:
        return {
            'z_bounds': list(zone_type.z_bounds),
            'width': zone_type.width,
            'y_padding': zone_type.y_padding,
            'z_padding': zone_type.z_padding,
        }

    @classmethod
    def zone_type_from_dict(cls, d: dict) -> ZoneType:
        return cls.PlanarZoneType(
            z_bounds=(d['z_bounds'][0], d['z_bounds'][1]),
            width=d.get('width'),
            y_padding=float(d.get('y_padding', 0.0)),
            z_padding=float(d.get('z_padding', 0.0)),
        )

    # ---- operations on the resolved zone -------------------------------------

    def to_dict(self) -> dict:
        zb = self.bounds
        return {
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

    def spatial_mask(self, xyz_map: np.ndarray) -> np.ndarray:
        """Axis-aligned bounding-box mask over the full planar zone extent."""
        zb = self.bounds
        return (
            (xyz_map[:, 0] >= zb.x_min) & (xyz_map[:, 0] <= zb.x_max)
            & (xyz_map[:, 1] >= zb.y_min) & (xyz_map[:, 1] <= zb.y_max)
            & (xyz_map[:, 2] >= zb.z_min) & (xyz_map[:, 2] <= zb.z_max)
        )

    def projective_mask(self, az, el, lidar_position, y_padding, z_padding) -> np.ndarray:
        """Angular window from the planar zone's 4 corners, padded INWARD."""
        zb = self.bounds
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

    def expected_fields(self, lidar_pos: np.ndarray) -> dict:
        """Flat plane at x with y/z extents — a box is exact for a plane."""
        zb = self.bounds
        lx, ly, lz = lidar_pos
        return {
            'x': zb.x_surface - lx,
            'y_min': zb.y_min - ly,
            'y_max': zb.y_max - ly,
            'z_min': zb.z_min - lz,
            'z_max': zb.z_max - lz,
        }

    # ---- visualization -------------------------------------------------------

    def build_markers(self, array, stamp, marker_id: int) -> int:
        """Layered thin slabs (geometry / intensity / noise) + a wireframe bbox."""
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        from lidar_zones.zones_api.zone_plugins.marker_helpers import (
            MATERIAL_COLORS,
            base_marker,
            color,
            intensity_color,
            noise_color,
        )

        zb = self.bounds
        z_center = (zb.z_min + zb.z_max) / 2.0
        z_height = zb.z_max - zb.z_min
        y_center = (zb.y_min + zb.y_max) / 2.0
        y_width = zb.y_max - zb.y_min

        layers = [
            ('geometry', zb.x_surface, MATERIAL_COLORS.get(zb.zone_config.color, color(0.8, 0.8, 0.8, 0.35))),
            ('intensity', zb.x_surface - 0.015, intensity_color(zb.zone_config.expected_intensity)),
            ('noise', zb.x_surface - 0.03, noise_color(zb.zone_config.noise_sigma_m)),
        ]
        for ns, x_pos, layer_color in layers:
            m = base_marker(ns, marker_id, Marker.CUBE, stamp)
            m.pose.position.x = x_pos
            m.pose.position.y = y_center
            m.pose.position.z = z_center
            m.scale.x = 0.005
            m.scale.y = y_width
            m.scale.z = z_height
            m.color = layer_color
            array.markers.append(m)
            marker_id += 1

        # ROI bounds — wireframe cube
        m = base_marker('roi_bounds', marker_id, Marker.LINE_LIST, stamp)
        m.scale.x = 0.005
        m.color = color(0.0, 1.0, 1.0, 0.9)
        corners = [
            [zb.x_min, zb.y_min, zb.z_min], [zb.x_max, zb.y_min, zb.z_min],
            [zb.x_max, zb.y_max, zb.z_min], [zb.x_min, zb.y_max, zb.z_min],
            [zb.x_min, zb.y_min, zb.z_max], [zb.x_max, zb.y_min, zb.z_max],
            [zb.x_max, zb.y_max, zb.z_max], [zb.x_min, zb.y_max, zb.z_max],
        ]
        for i, j in [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]:
            for idx in (i, j):
                pt = Point()
                pt.x, pt.y, pt.z = corners[idx]
                m.points.append(pt)
        array.markers.append(m)
        return marker_id + 1

    # ---- generation-time -----------------------------------------------------

    @classmethod
    def construct_urdf_link(cls, props: dict):
        """A planar zone renders as a thin box link (depth × length × height)."""
        name = props.get('frame', props.get('name', 'planar_zone'))
        depth = float(props.get('depth', 0.0))
        length = float(props.get('length', 0.0))
        height = float(props.get('height', 0.0))
        r, g, b = [c / 255.0 for c in props.get('color', [128, 128, 128])]

        link = Link(name=name)
        material = LinkMaterial(name=f'{name}_material')
        material.color = Color()
        material.color.rgba = [r, g, b, 1.0]
        visual = Visual()
        visual.geometry = Box(size=[depth, length, height])
        visual.material = material
        link.add_aggregate('visual', visual)
        return link

    @classmethod
    def roi_fields(cls, props: dict) -> dict:
        """Planar ROI fields: width + z_bounds (z_offset .. z_offset+height) + y/z padding."""
        z_offset = props.get('z_offset', 0.0)
        height = props.get('height', 0.0)
        return {
            'width': props.get('length', 0.0),
            'z_bounds': [z_offset, round(z_offset + height, 6)],
            'y_padding': float(props.get('y_padding', 0.0)),
            'z_padding': float(props.get('z_padding', 0.0)),
        }

    @classmethod
    def lateral_half_extent(cls, props: dict) -> float:
        """A planar zone's lateral half-extent is half its width along Y."""
        return float(props.get('length', 0.0)) / 2.0
