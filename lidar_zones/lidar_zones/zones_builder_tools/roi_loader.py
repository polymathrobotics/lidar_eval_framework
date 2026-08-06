# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from pathlib import Path

import yaml

# Zone data structs + the geometry-type parser registry live in zones_api.
# Re-exported here so existing `from roi_loader import ZoneConfig/ROIConfig` works.
from lidar_zones.zones_api import ZoneEngine
from lidar_zones.zones_api.profile_types import (  # noqa: F401  (ROIConfig/ZoneConfig re-exported)
    GEOMETRY_SURFACE,
    ROIConfig,
    ZoneConfig,
)


KNOWN_COLORS: dict[str, float] = {
    'white': 220.0,
    'green': 80.0,
    'turquoise': 160.0,  # RGB (64, 224, 208) — ~65% brightness
    'grey': 120.0,
    'black': 20.0,
    'red': 150.0,
    'blue': 100.0,
    'yellow': 180.0,
}

SURFACE_NOISE_SIGMA: dict[str, float] = {
    'planar': 0.002,
    'curved': 0.005,
}


class ROILoader:
    """Loads and validates an ROI configuration from a YAML file."""

    def __init__(self) -> None:
        # Routes each zone's geometry to its plugin for parsing.
        self._engine = ZoneEngine()

    def load(self, config_path: str) -> ROIConfig:
        """Load and validate roi.yaml, returning a parsed ROIConfig.

        Args:
            config_path: Absolute path to the roi.yaml file.

        Returns:
            Parsed and validated ROIConfig.

        Raises:
            FileNotFoundError: If the config file does not exist.
            ValueError: If the config is malformed or contains invalid values.
        """
        path = Path(config_path)
        if not path.exists():
            raise FileNotFoundError(f'ROI config file not found: {config_path}')

        try:
            with path.open('r') as f:
                raw = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            raise ValueError(f'Malformed YAML in {config_path}: {exc}') from exc

        if raw is None or not isinstance(raw, dict):
            raise ValueError(f'ROI config {config_path} is empty or not a mapping')

        raw_zones = raw.get('zones')
        if not isinstance(raw_zones, list) or len(raw_zones) == 0:
            raise ValueError('ROI config must contain a non-empty "zones" list')

        seen_names: set[str] = set()
        zones: list[ZoneConfig] = []

        for i, zone_raw in enumerate(raw_zones):
            if not isinstance(zone_raw, dict):
                raise ValueError(f'Zone at index {i} must be a mapping, got {type(zone_raw).__name__}')

            zone = self._parse_zone(zone_raw, i)

            if zone.name in seen_names:
                raise ValueError(f'Duplicate zone name: "{zone.name}"')
            seen_names.add(zone.name)

            zones.append(zone)

        return ROIConfig(zones=zones)

    def _parse_zone(self, raw: dict, index: int) -> ZoneConfig:
        """Parse and validate a single zone entry.

        Validates the shared fields here; the YAML `type` field selects the
        geometry, and the geometry-specific fields are parsed by that geometry's
        plugin (resolved via the ZoneEngine).
        """
        location = f'zone at index {index}'

        name = raw.get('name')
        if not name or not isinstance(name, str):
            raise ValueError(f'Missing or invalid "name" field in {location}')
        location = f'zone "{name}"'

        frame = raw.get('frame')
        if not frame or not isinstance(frame, str):
            raise ValueError(f'Missing or invalid "frame" field in {location}')

        geometry = raw.get('type')
        if not geometry or not isinstance(geometry, str):
            raise ValueError(f'Missing or invalid "type" field in {location}')

        if geometry not in GEOMETRY_SURFACE:
            raise ValueError(
                f'Unknown zone type "{geometry}" in {location}. '
                f'Known types: {sorted(GEOMETRY_SURFACE.keys())}'
            )

        color = raw.get('color')
        if not color or not isinstance(color, str):
            raise ValueError(f'Missing or invalid "color" field in {location}')

        if color not in KNOWN_COLORS:
            raise ValueError(
                f'Unknown color "{color}" in {location}. '
                f'Known colors: {sorted(KNOWN_COLORS.keys())}'
            )

        zone_type = self._engine.plugin_for(geometry).parse_zone_type(raw, location)

        return ZoneConfig(
            name=name,
            frame=frame,
            color=color,
            expected_intensity=KNOWN_COLORS[color],
            noise_sigma_m=SURFACE_NOISE_SIGMA[GEOMETRY_SURFACE[geometry]],
            zone_type=zone_type,
        )
