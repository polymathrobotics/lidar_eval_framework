# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""The zone engine — routes zones to their geometry plugin.

`ZoneEngine` is the single entry point consumers use (zones node, filter,
controller). At construction it reads `zones_types_registry.yaml`, dynamically
imports each listed plugin, and builds its routing tables. It never imports a
concrete plugin itself — adding a geometry means adding a `zone_plugins/*.py` and
a row in the YAML; the engine is untouched.

The **geometry label** (e.g. 'planar') lives only in the registry, not on the
plugin. The engine owns the label↔plugin mapping: it tags serialized output with
the geometry and routes deserialization by it, and exposes `geometry_of(...)` for
consumers that need the label (e.g. the viz ExpectedZone builder).
"""

from __future__ import annotations

import importlib
import json
import os

import numpy as np
import yaml

from lidar_zones.zones_api.profile_types import (
    BaselineProfiles,
    FramePose,
    ROIConfig,
    ZoneBounds,
    ZoneConfig,
    ZoneType,
)
from lidar_zones.zones_api.zone_plugin_api import ZoneTypePlugin

_REGISTRY_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'zones_types_registry.yaml')
_PLUGINS_PACKAGE = 'lidar_zones.zones_api.zone_plugins'


class ZoneEngine:
    """Discover plugins from the registry YAML, then route/instantiate them."""

    def __init__(self, registry_path: str = _REGISTRY_YAML) -> None:
        entries = self._load_plugins(registry_path)     # [(geometry, plugin_cls), ...]
        self._by_geometry = {geo: cls for geo, cls in entries}
        self._by_zone_type = {cls.zone_type_cls: cls for _, cls in entries}
        self._by_bounds = {cls.bounds_cls: cls for _, cls in entries}
        # reverse maps: struct class -> geometry label (used to tag serialized output)
        self._geo_by_zone_type = {cls.zone_type_cls: geo for geo, cls in entries}
        self._geo_by_bounds = {cls.bounds_cls: geo for geo, cls in entries}

    @staticmethod
    def _load_plugins(registry_path: str) -> list[tuple[str, type[ZoneTypePlugin]]]:
        """Import every plugin listed in the registry YAML, returning
        (geometry, plugin_class) pairs. The geometry label comes from the YAML —
        the plugin itself doesn't declare one."""
        with open(registry_path, 'r') as f:
            registry = yaml.safe_load(f) or {}

        entries: list[tuple[str, type[ZoneTypePlugin]]] = []
        for entry in registry.get('zone_plugins', []):
            geometry = entry['ZoneType']
            module = importlib.import_module(f'{_PLUGINS_PACKAGE}.{entry["executable"]}')
            plugin_cls = getattr(module, entry['Class'])
            if not (isinstance(plugin_cls, type) and issubclass(plugin_cls, ZoneTypePlugin)):
                raise TypeError(f'{entry["Class"]} is not a ZoneTypePlugin subclass')
            entries.append((geometry, plugin_cls))
        return entries

    # ---- routing -------------------------------------------------------------

    def plugin_for(self, geometry: str) -> type[ZoneTypePlugin]:
        """The plugin class for a geometry string (call its classmethods: parse,
        from_dict, zone_type_to_dict, construct_urdf_link)."""
        try:
            return self._by_geometry[geometry]
        except KeyError:
            raise KeyError(f'No zone plugin for geometry {geometry!r}. '
                           f'Known: {sorted(self._by_geometry)}') from None

    def geometry_of(self, bounds: ZoneBounds) -> str:
        """The geometry label for a resolved ZoneBounds (for consumers that must
        tag output, e.g. the viz ExpectedZone builder)."""
        try:
            return self._geo_by_bounds[type(bounds)]
        except KeyError:
            raise KeyError(f'No geometry registered for ZoneBounds {type(bounds).__name__}') from None

    def _for_zone_type(self, zone_type: ZoneType) -> type[ZoneTypePlugin]:
        try:
            return self._by_zone_type[type(zone_type)]
        except KeyError:
            raise KeyError(f'No zone plugin for ZoneType {type(zone_type).__name__}') from None

    def _for_bounds(self, bounds: ZoneBounds) -> type[ZoneTypePlugin]:
        try:
            return self._by_bounds[type(bounds)]
        except KeyError:
            raise KeyError(f'No zone plugin for ZoneBounds {type(bounds).__name__}') from None

    # ---- instantiation -------------------------------------------------------

    def build(self, zone_cfg: ZoneConfig, pose: FramePose, lidar_pos: np.ndarray) -> ZoneTypePlugin:
        return self._for_zone_type(zone_cfg.zone_type).build(zone_cfg, pose, lidar_pos)

    def register_zones(
        self,
        roi_config: ROIConfig,
        frame_poses: dict[str, FramePose],
        lidar_pos: np.ndarray,
    ) -> dict[str, ZoneTypePlugin]:
        """One plugin instance per zone in the ROI config, keyed by zone name."""
        return {z.name: self.build(z, frame_poses[z.frame], lidar_pos) for z in roi_config.zones}

    def wrap(self, bounds: ZoneBounds) -> ZoneTypePlugin:
        """Wrap a resolved (e.g. deserialized) ZoneBounds into its plugin."""
        return self._for_bounds(bounds)(bounds)

    def roi_fields(self, geometry: str, props: dict) -> dict:
        """Build a zone's geometry-specific ROI fields (for roi.yaml) via its plugin."""
        return self.plugin_for(geometry).roi_fields(props)

    def lateral_half_extent(self, geometry: str, props: dict) -> float:
        """A zone's lateral (Y) half-extent via its plugin (for pan sweep + placement)."""
        return self.plugin_for(geometry).lateral_half_extent(props)

    # ---- profile (de)serialization (orchestration across plugins) -----------

    def profiles_to_json(self, profiles: BaselineProfiles) -> str:
        return json.dumps({
            'zone_bounds': [self._zone_bounds_to_dict(zb) for zb in profiles.zone_bounds],
            'lidar_position': profiles.lidar_position.tolist(),
        })

    def profiles_from_json(self, data: str) -> BaselineProfiles:
        d = json.loads(data)
        return BaselineProfiles(
            zone_bounds=[self._zone_bounds_from_dict(zb) for zb in d['zone_bounds']],
            lidar_position=np.array(d['lidar_position'], dtype=np.float64),
        )

    def _zone_bounds_to_dict(self, zb: ZoneBounds) -> dict:
        # The engine tags the geometry (plugins don't carry the label).
        out = {
            'name': zb.name,
            'geometry': self._geo_by_bounds[type(zb)],
            'zone_config': self._zone_config_to_dict(zb.zone_config),
        }
        out.update(self.wrap(zb).to_dict())        # per-geometry fields via the plugin
        return out

    def _zone_bounds_from_dict(self, d: dict) -> ZoneBounds:
        zone_config = self._zone_config_from_dict(d['zone_config'])
        return self.plugin_for(d['geometry']).from_dict(d, zone_config).bounds

    def _zone_config_to_dict(self, zc: ZoneConfig) -> dict:
        zt = zc.zone_type
        zone_type_dict = {
            'geometry': self._geo_by_zone_type[type(zt)],
            **self._for_zone_type(zt).zone_type_to_dict(zt),
        }
        return {
            'name': zc.name,
            'frame': zc.frame,
            'color': zc.color,
            'expected_intensity': zc.expected_intensity,
            'noise_sigma_m': zc.noise_sigma_m,
            'zone_type': zone_type_dict,
        }

    def _zone_config_from_dict(self, d: dict) -> ZoneConfig:
        zt_d = d['zone_type']
        return ZoneConfig(
            name=d['name'],
            frame=d['frame'],
            color=d['color'],
            expected_intensity=d['expected_intensity'],
            noise_sigma_m=d['noise_sigma_m'],
            zone_type=self.plugin_for(zt_d['geometry']).zone_type_from_dict(zt_d),
        )
