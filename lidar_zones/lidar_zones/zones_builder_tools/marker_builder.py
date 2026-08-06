# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""Builds a MarkerArray visualizing all baseline profiles.

Per-zone markers are geometry-specific and owned by each geometry plugin
(`build_markers`); this driver just routes each resolved zone through the
ZoneEngine, so adding a new zone geometry needs no edits here.
"""

from __future__ import annotations

from visualization_msgs.msg import Marker, MarkerArray

from lidar_zones.zones_api import ZoneEngine
from lidar_zones.zones_api.profile_types import BaselineProfiles


class MarkerBuilder:
    """Builds a MarkerArray visualizing all baseline profiles."""

    def __init__(self) -> None:
        # Routes each resolved zone to its geometry plugin for marker building.
        self._engine = ZoneEngine()

    def build(self, profiles: BaselineProfiles, stamp) -> MarkerArray:
        """Build a complete MarkerArray for the given profiles.

        Each zone's markers are produced by its geometry plugin's `build_markers`.
        """
        array = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        array.markers.append(delete_all)

        marker_id = 0
        for zb in profiles.zone_bounds:
            marker_id = self._engine.wrap(zb).build_markers(array, stamp, marker_id)

        return array
