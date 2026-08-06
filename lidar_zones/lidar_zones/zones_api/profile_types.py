# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""Shared zone/profile data vocabulary.

The single home for the geometry-agnostic data types every consumer speaks —
the ZoneType/ZoneBounds bases, zone/ROI config, TF pose, and the profile structs
produced by ProfileBuilder (in zones_builder_tools). Concrete per-geometry
subclasses live in plugins/<geometry>.py. Intentionally ROS-free (numpy only).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


# the bottom should be moved zones engine or something

# Maps a zone's geometry (the YAML `type` field) to the surface-noise model used
# to derive ZoneConfig.noise_sigma_m. Each plugin's geometry key appears here.
GEOMETRY_SURFACE: dict[str, str] = {
    'planar': 'planar',
    'cylindrical': 'curved',
}


class ZoneType:
    """Base for geometry-specific ROI type params (parsed from YAML)."""


class ZoneBounds:
    """Base for geometry-specific resolved spatial bounds."""


@dataclass
class FramePose:
    """Pose of a TF frame expressed in the map frame."""

    position: np.ndarray   # shape (3,): [x, y, z]
    rotation: np.ndarray   # shape (3, 3): rotation matrix


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
