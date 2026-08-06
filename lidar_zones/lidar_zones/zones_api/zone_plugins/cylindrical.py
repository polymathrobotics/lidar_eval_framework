# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""Cylindrical zone geometry plugin — everything about a cylindrical ROI zone.

The geometry's data structs (ZoneType + ZoneBounds) are nested inside the plugin
class, so a single class holds the geometry's data *and* behavior.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from urdf_parser_py.urdf import Box, Color, Cylinder, Link, LinkMaterial, Visual

from lidar_zones.zones_api.profile_types import FramePose, ZoneBounds, ZoneConfig, ZoneType
from lidar_zones.zones_api.zone_plugin_api import ZoneTypePlugin


class CylindricalZonePlugin(ZoneTypePlugin):
    """Plugin for cylindrical zones. `self.bounds` is a CylindricalZoneBounds."""

    # ---- data structs (nested: the plugin owns its geometry's structs) -------

    @dataclass
    class CylindricalZoneType(ZoneType):
        """Geometry parameters specific to a cylindrical ROI zone."""

        height: float
        radius: float
        # radius_padding/height_padding: projective filter applies INWARD; spatial
        # applies OUTWARD (radius_padding→Y, height_padding→Z). outward_radius_padding
        # is spatial-only, growing X (fwd/back).
        radius_padding: float = 0.0
        height_padding: float = 0.0
        outward_radius_padding: float = 0.0

    @dataclass
    class CylindricalZoneBounds(ZoneBounds):
        """Resolved spatial bounds for a cylindrical ROI zone (axis vertical, +Z)."""

        name: str
        zone_config: ZoneConfig
        center_x: float
        center_y: float
        radius: float
        x_surface: float         # nearest face X toward the lidar (center_x - radius)
        expected_depth_m: float
        z_min: float = 0.0
        z_max: float = 0.0
        radius_padding: float = 0.0
        height_padding: float = 0.0
        outward_radius_padding: float = 0.0

    zone_type_cls = CylindricalZoneType
    bounds_cls = CylindricalZoneBounds

    # ---- construction --------------------------------------------------------

    @classmethod
    def parse_zone_type(cls, raw: dict, location: str) -> "CylindricalZonePlugin.CylindricalZoneType":
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
            raise ValueError(f'"radius_padding" ({radius_padding}) must be < radius ({radius}) in {location}')
        if 2.0 * height_padding >= height:
            raise ValueError(
                f'"height_padding" ({height_padding}) too large in {location}: 2x must be < height ({height})'
            )

        outward_radius_padding = float(raw.get('outward_radius_padding', 0.0))
        if outward_radius_padding < 0.0:
            raise ValueError(f'"outward_radius_padding" must be >= 0 in {location}')

        return cls.CylindricalZoneType(
            height=height, radius=radius, radius_padding=radius_padding,
            height_padding=height_padding, outward_radius_padding=outward_radius_padding,
        )

    @classmethod
    def build(cls, zone_cfg: ZoneConfig, pose: FramePose, lidar_pos: np.ndarray) -> "CylindricalZonePlugin":
        """Resolve a cylindrical zone's FULL bounds from radius/height + TF pose."""
        zt: CylindricalZonePlugin.CylindricalZoneType = zone_cfg.zone_type
        center_x = pose.position[0]
        center_y = pose.position[1]
        center_z = pose.position[2]

        half_height = zt.height / 2.0
        z_min = center_z - half_height
        z_max = center_z + half_height

        x_surface = center_x - zt.radius  # near face toward the lidar (forward = +x)
        expected_depth_m = x_surface - lidar_pos[0]

        bounds = cls.CylindricalZoneBounds(
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
        return cls(bounds)

    @classmethod
    def from_dict(cls, d: dict, zone_config: ZoneConfig) -> "CylindricalZonePlugin":
        bounds = cls.CylindricalZoneBounds(
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
        return cls(bounds)

    @classmethod
    def zone_type_to_dict(cls, zone_type: ZoneType) -> dict:
        return {
            'height': zone_type.height,
            'radius': zone_type.radius,
            'radius_padding': zone_type.radius_padding,
            'height_padding': zone_type.height_padding,
        }

    @classmethod
    def zone_type_from_dict(cls, d: dict) -> "CylindricalZonePlugin.CylindricalZoneType":
        return cls.CylindricalZoneType(
            height=d['height'],
            radius=d['radius'],
            radius_padding=float(d.get('radius_padding', 0.0)),
            height_padding=float(d.get('height_padding', 0.0)),
        )

    # ---- operations on the resolved zone -------------------------------------

    def to_dict(self) -> dict:
        zb = self.bounds
        return {
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

    def spatial_mask(self, xyz_map: np.ndarray) -> np.ndarray:
        """Axis-aligned bounding prism enclosing the whole cylinder, grown OUTWARD."""
        zb = self.bounds
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

    def projective_mask(self, az, el, lidar_position, y_padding, z_padding) -> np.ndarray:
        """Angular cone from the cylinder silhouette, padded INWARD.

        y_padding/z_padding are unused (the cylinder carries its own
        radius_padding/height_padding); they keep the interface signature uniform.
        """
        zb = self.bounds
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

    def expected_fields(self, lidar_pos: np.ndarray) -> dict:
        """Cylinder axis (center_x/center_y) + radius + z span, lidar-relative."""
        zb = self.bounds
        lx, ly, lz = lidar_pos
        return {
            'center_x': zb.center_x - lx,
            'center_y': zb.center_y - ly,
            'radius': zb.radius,
            'z_min': zb.z_min - lz,
            'z_max': zb.z_max - lz,
        }

    # ---- visualization -------------------------------------------------------

    def build_markers(self, array, stamp, marker_id: int) -> int:
        """Near half-cylinder shell (the 180deg arc facing the lidar) + a half-arc
        wireframe + the axis-aligned spatial bounding prism.

        The lidar sits at the map origin, so the near half faces the origin: its
        arc is centered on the bearing from the axis back toward the origin.
        Rendered as a TRIANGLE_LIST (RViz has no half-cylinder primitive).
        """
        import math

        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker

        from lidar_zones.zones_api.zone_plugins.marker_helpers import (
            MATERIAL_COLORS,
            base_marker,
            color,
        )

        zb = self.bounds
        segments = 24
        # Bearing from the cylinder axis toward the lidar (map origin), and the
        # near semicircle [center - 90deg, center + 90deg] around it.
        center_bearing = math.atan2(-zb.center_y, -zb.center_x)
        angles = [
            center_bearing - (math.pi / 2.0) + math.pi * i / segments
            for i in range(segments + 1)
        ]
        ring = [(zb.center_x + zb.radius * math.cos(a), zb.center_y + zb.radius * math.sin(a)) for a in angles]

        # Geometry — near half shell as a triangle strip between z_min and z_max.
        m = base_marker('geometry', marker_id, Marker.TRIANGLE_LIST, stamp)
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = MATERIAL_COLORS.get(zb.zone_config.color, color(0.8, 0.8, 0.8, 0.35))
        for i in range(segments):
            (x0, y0), (x1, y1) = ring[i], ring[i + 1]
            quad = [
                (x0, y0, zb.z_min), (x1, y1, zb.z_min), (x1, y1, zb.z_max),  # tri 1
                (x0, y0, zb.z_min), (x1, y1, zb.z_max), (x0, y0, zb.z_max),  # tri 2
            ]
            for px, py, pz in quad:
                pt = Point()
                pt.x, pt.y, pt.z = float(px), float(py), float(pz)
                m.points.append(pt)
        array.markers.append(m)
        marker_id += 1

        # ROI bounds — half-arc wireframe: top + bottom arcs + vertical end edges.
        m = base_marker('roi_bounds', marker_id, Marker.LINE_LIST, stamp)
        m.scale.x = 0.005
        m.color = color(0.0, 1.0, 1.0, 0.9)
        for z in (zb.z_min, zb.z_max):
            for i in range(segments):
                for (px, py) in (ring[i], ring[i + 1]):
                    pt = Point()
                    pt.x, pt.y, pt.z = float(px), float(py), float(z)
                    m.points.append(pt)
        for (px, py) in (ring[0], ring[-1]):
            for z in (zb.z_min, zb.z_max):
                pt = Point()
                pt.x, pt.y, pt.z = float(px), float(py), float(z)
                m.points.append(pt)
        array.markers.append(m)
        marker_id += 1

        # Spatial bounding prism — the axis-aligned box the spatial filter actually
        # keeps: the cylinder's full extent grown OUTWARD per axis (X by
        # outward_radius_padding, Y by radius_padding, Z by height_padding). Drawn
        # translucent so the base half-shell stays visible inside it.
        m = base_marker('spatial_prism', marker_id, Marker.CUBE, stamp)
        m.pose.position.x = float(zb.center_x)
        m.pose.position.y = float(zb.center_y)
        m.pose.position.z = float((zb.z_min + zb.z_max) / 2.0)
        m.scale.x = float(2.0 * (zb.radius + zb.outward_radius_padding))
        m.scale.y = float(2.0 * (zb.radius + zb.radius_padding))
        m.scale.z = float((zb.z_max - zb.z_min) + 2.0 * zb.height_padding)
        m.color = color(1.0, 0.6, 0.0, 0.18)
        array.markers.append(m)
        return marker_id + 1

    # ---- generation-time -----------------------------------------------------

    @classmethod
    def construct_urdf_link(cls, props: dict):
        """`position: forward` renders as a Cylinder; otherwise a flat box facade."""
        name = props.get('frame', props.get('name', 'cylindrical_zone'))
        height = float(props.get('height', 0.0))
        radius = float(props.get('radius', 0.0))
        r, g, b = [c / 255.0 for c in props.get('color', [128, 128, 128])]

        link = Link(name=name)
        material = LinkMaterial(name=f'{name}_material')
        material.color = Color()
        visual = Visual()
        if props.get('position', 'forward') == 'forward':
            material.color.rgba = [r, g, b, 0.4]   # transparent so the cloud shows through
            visual.geometry = Cylinder(length=height, radius=radius)
        else:
            material.color.rgba = [r, g, b, 1.0]
            depth = float(props.get('depth', 0.0))
            visual.geometry = Box(size=[depth, radius * 2.0, height])
        visual.material = material
        link.add_aggregate('visual', visual)
        return link

    @classmethod
    def roi_fields(cls, props: dict) -> dict:
        """Cylindrical ROI fields: radius/height + radial/axial paddings.

        Padding falls back to the planar y/z keys so configs predating the
        radius/height padding split still resolve (radial <- y, axial <- z)."""
        return {
            'height': float(props.get('height', 0.0)),
            'radius': float(props.get('radius', 0.0)),
            'radius_padding': float(props.get('radius_padding', props.get('y_padding', 0.0))),
            'height_padding': float(props.get('height_padding', props.get('z_padding', 0.0))),
            'outward_radius_padding': float(props.get('outward_radius_padding', 0.0)),
        }

    @classmethod
    def lateral_half_extent(cls, props: dict) -> float:
        """A cylindrical zone's lateral half-extent is its radius."""
        return float(props.get('radius', 0.0))
