# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

from __future__ import annotations

import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from lidar_transforms.tools.profile_builder import BaselineProfiles, NoiseRegion
from lidar_transforms.tools.zones_utilities import (
    MARKER_BUILDERS,
    CylindricalZoneBounds,
    PlanarZoneBounds,
)


# ---------------------------------------------------------------------------
# Color helpers
# ---------------------------------------------------------------------------

def _color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = r, g, b, a
    return c


_MATERIAL_COLORS: dict[str, ColorRGBA] = {
    'white':  _color(1.0, 1.0, 1.0, 0.35),
    'green':      _color(0.1, 0.8, 0.1, 0.35),
    'turquoise':  _color(0.251, 0.878, 0.816, 0.35),  # RGB (64, 224, 208)
    'grey':   _color(0.5, 0.5, 0.5, 0.35),
    'black':  _color(0.1, 0.1, 0.1, 0.35),
    'red':    _color(0.9, 0.1, 0.1, 0.35),
    'blue':   _color(0.1, 0.1, 0.9, 0.35),
    'yellow': _color(0.9, 0.9, 0.1, 0.35),
}


def _intensity_color(expected_intensity: float) -> ColorRGBA:
    if expected_intensity > 180.0:
        return _color(1.0, 1.0, 1.0, 0.5)
    elif expected_intensity >= 100.0:
        return _color(1.0, 1.0, 0.3, 0.5)
    return _color(1.0, 0.4, 0.0, 0.5)


def _noise_color(sigma_m: float) -> ColorRGBA:
    if sigma_m < 0.003:
        return _color(0.0, 0.3, 1.0, 0.4)    # blue  — low noise
    elif sigma_m <= 0.01:
        return _color(1.0, 0.9, 0.0, 0.4)    # yellow — medium
    return _color(1.0, 0.5, 0.0, 0.4)         # orange — high


def _base_marker(ns: str, marker_id: int, marker_type: int, stamp) -> Marker:
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = marker_type
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


# ---------------------------------------------------------------------------
# Per-zone marker builders — registered into the central zones_utilities map.
# Each returns the next marker id. The marker bodies live here (ROS-dependent),
# but the routing lives in zones_utilities.MARKER_BUILDERS.
# ---------------------------------------------------------------------------

@MARKER_BUILDERS.register(PlanarZoneBounds)
def planar_zone_markers(array: MarkerArray, zb: PlanarZoneBounds, stamp, marker_id: int) -> int:
    """Layered thin slabs (geometry / intensity / noise) + a wireframe bbox."""
    z_center = (zb.z_min + zb.z_max) / 2.0
    z_height = zb.z_max - zb.z_min
    y_center = (zb.y_min + zb.y_max) / 2.0
    y_width = zb.y_max - zb.y_min

    layers = [
        ('geometry', zb.x_surface, _MATERIAL_COLORS.get(zb.zone_config.color, _color(0.8, 0.8, 0.8, 0.35))),
        ('intensity', zb.x_surface - 0.015, _intensity_color(zb.zone_config.expected_intensity)),
        ('noise', zb.x_surface - 0.03, _noise_color(zb.zone_config.noise_sigma_m)),
    ]
    for ns, x_pos, color in layers:
        m = _base_marker(ns, marker_id, Marker.CUBE, stamp)
        m.pose.position.x = x_pos
        m.pose.position.y = y_center
        m.pose.position.z = z_center
        m.scale.x = 0.005
        m.scale.y = y_width
        m.scale.z = z_height
        m.color = color
        array.markers.append(m)
        marker_id += 1

    # ROI bounds — wireframe cube
    m = _base_marker('roi_bounds', marker_id, Marker.LINE_LIST, stamp)
    m.scale.x = 0.005
    m.color = _color(0.0, 1.0, 1.0, 0.9)
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


@MARKER_BUILDERS.register(CylindricalZoneBounds)
def cylindrical_zone_markers(array: MarkerArray, zb: CylindricalZoneBounds, stamp, marker_id: int) -> int:
    """Near half-cylinder shell (the 180deg arc facing the lidar) + a half-arc wireframe.

    The lidar sits at the map origin, so the near half faces the origin: its arc
    is centered on the bearing from the axis back toward the origin. Rendered as
    a TRIANGLE_LIST (RViz has no half-cylinder primitive).
    """
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
    m = _base_marker('geometry', marker_id, Marker.TRIANGLE_LIST, stamp)
    m.scale.x = m.scale.y = m.scale.z = 1.0
    m.color = _MATERIAL_COLORS.get(zb.zone_config.color, _color(0.8, 0.8, 0.8, 0.35))
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

    # ROI bounds — half-arc wireframe: top + bottom arcs + vertical edges at the ends.
    m = _base_marker('roi_bounds', marker_id, Marker.LINE_LIST, stamp)
    m.scale.x = 0.005
    m.color = _color(0.0, 1.0, 1.0, 0.9)
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
    m = _base_marker('spatial_prism', marker_id, Marker.CUBE, stamp)
    m.pose.position.x = float(zb.center_x)
    m.pose.position.y = float(zb.center_y)
    m.pose.position.z = float((zb.z_min + zb.z_max) / 2.0)
    m.scale.x = float(2.0 * (zb.radius + zb.outward_radius_padding))
    m.scale.y = float(2.0 * (zb.radius + zb.radius_padding))
    m.scale.z = float((zb.z_max - zb.z_min) + 2.0 * zb.height_padding)
    m.color = _color(1.0, 0.6, 0.0, 0.18)
    array.markers.append(m)
    return marker_id + 1


# ---------------------------------------------------------------------------
# MarkerBuilder
# ---------------------------------------------------------------------------

class MarkerBuilder:
    """Builds a MarkerArray visualizing all baseline profiles."""

    def build(self, profiles: BaselineProfiles, stamp) -> MarkerArray:
        """Build a complete MarkerArray for the given profiles.

        Per-zone geometry markers are routed through zones_utilities.MARKER_BUILDERS;
        noise-region markers are geometry-independent and built here.
        """
        array = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        array.markers.append(delete_all)

        marker_id = 0

        for zb in profiles.zone_bounds:
            marker_id = MARKER_BUILDERS.for_obj(zb)(array, zb, stamp, marker_id)

        return array

    def _edge_noise_marker(self, array: MarkerArray, nr: NoiseRegion, stamp, marker_id: int) -> int:
        return self._noise_region_marker(array, nr, stamp, marker_id, _color(1.0, 0.6, 0.0, 0.7))

    def _corner_noise_marker(self, array: MarkerArray, nr: NoiseRegion, stamp, marker_id: int) -> int:
        return self._noise_region_marker(array, nr, stamp, marker_id, _color(1.0, 0.0, 0.0, 0.8))

    def _noise_region_marker(
        self, array: MarkerArray, nr: NoiseRegion, stamp, marker_id: int, color: ColorRGBA,
    ) -> int:
        z_center = (nr.z_min + nr.z_max) / 2.0
        z_height = nr.z_max - nr.z_min
        m = _base_marker('noise', marker_id, Marker.CYLINDER, stamp)
        m.pose.position.x = float(nr.center[0])
        m.pose.position.y = float(nr.center[1])
        m.pose.position.z = z_center
        m.scale.x = nr.radius * 2.0
        m.scale.y = nr.radius * 2.0
        m.scale.z = z_height
        m.color = color
        array.markers.append(m)
        return marker_id + 1
