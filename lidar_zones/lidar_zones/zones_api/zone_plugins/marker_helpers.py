"""RViz marker primitives shared by the geometry plugins' `build_markers`.

This module is ROS-dependent (visualization_msgs / std_msgs). Plugins import it
*inside* their `build_markers` method, never at module top level, so pure
consumers like the filter node never drag in the visualization message types.
"""

from __future__ import annotations

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


def color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = r, g, b, a
    return c


# Zone color name -> translucent fill used for the "geometry" layer.
MATERIAL_COLORS: dict[str, ColorRGBA] = {
    'white':      color(1.0, 1.0, 1.0, 0.35),
    'green':      color(0.1, 0.8, 0.1, 0.35),
    'turquoise':  color(0.251, 0.878, 0.816, 0.35),  # RGB (64, 224, 208)
    'grey':       color(0.5, 0.5, 0.5, 0.35),
    'black':      color(0.1, 0.1, 0.1, 0.35),
    'red':        color(0.9, 0.1, 0.1, 0.35),
    'blue':       color(0.1, 0.1, 0.9, 0.35),
    'yellow':     color(0.9, 0.9, 0.1, 0.35),
}


def intensity_color(expected_intensity: float) -> ColorRGBA:
    if expected_intensity > 180.0:
        return color(1.0, 1.0, 1.0, 0.5)
    elif expected_intensity >= 100.0:
        return color(1.0, 1.0, 0.3, 0.5)
    return color(1.0, 0.4, 0.0, 0.5)


def noise_color(sigma_m: float) -> ColorRGBA:
    if sigma_m < 0.003:
        return color(0.0, 0.3, 1.0, 0.4)    # blue   — low noise
    elif sigma_m <= 0.01:
        return color(1.0, 0.9, 0.0, 0.4)    # yellow — medium
    return color(1.0, 0.5, 0.0, 0.4)         # orange — high


def base_marker(ns: str, marker_id: int, marker_type: int, stamp) -> Marker:
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp = stamp
    m.ns = ns
    m.id = marker_id
    m.type = marker_type
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m
