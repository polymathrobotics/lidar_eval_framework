# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""Shared zone vocabulary + plugin engine.

Layout:
  profile_types.py    — data types (ZoneType/ZoneBounds bases, ZoneConfig, profiles) + GEOMETRY_SURFACE
  zone_plugin_api.py  — ZoneTypePlugin: the contract each geometry implements
  zone_engine.py      — ZoneEngine: routes a zone to its plugin + (de)serializes profiles
  zone_plugins/       — one module per geometry (planar, cylindrical, …)

Raw transform math (quat→matrix, apply_transform, …) lives in the
`lidar_transforms` package, not here.

Consumers import `ZoneEngine` to get plugins and `ZoneTypePlugin` for typing;
the concrete plugin classes are known only to the engine.
"""

from lidar_zones.zones_api.zone_engine import ZoneEngine  # noqa: F401
from lidar_zones.zones_api.zone_plugin_api import ZoneTypePlugin  # noqa: F401
