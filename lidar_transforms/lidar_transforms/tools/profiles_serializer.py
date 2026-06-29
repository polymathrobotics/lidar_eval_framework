# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

from __future__ import annotations

import json

import numpy as np

from lidar_transforms.tools.profile_builder import BaselineProfiles, NoiseRegion
from lidar_transforms.tools.zones_utilities import (
    ZONE_BOUNDS_FROM_DICT,
    ZONE_BOUNDS_TO_DICT,
    ZONE_TYPE_FROM_DICT,
    ZONE_TYPE_TO_DICT,
    ZoneConfig,
)


def profiles_to_json(profiles: BaselineProfiles) -> str:
    """Serialize a BaselineProfiles object to a JSON string."""
    return json.dumps({
        'zone_bounds': [_zone_bounds_to_dict(zb) for zb in profiles.zone_bounds],
        'lidar_position': profiles.lidar_position.tolist(),
    })


def profiles_from_json(data: str) -> BaselineProfiles:
    """Deserialize a JSON string back into a BaselineProfiles object."""
    d = json.loads(data)
    return BaselineProfiles(
        zone_bounds=[_zone_bounds_from_dict(zb) for zb in d['zone_bounds']],
        lidar_position=np.array(d['lidar_position'], dtype=np.float64),
    )


# ---------------------------------------------------------------------------
# Private helpers — geometry-specific (de)serialization is routed through the
# zones_utilities registries; only the shared fields are handled here.
# ---------------------------------------------------------------------------

def _zone_config_to_dict(zc: ZoneConfig) -> dict:
    return {
        'name': zc.name,
        'frame': zc.frame,
        'color': zc.color,
        'expected_intensity': zc.expected_intensity,
        'noise_sigma_m': zc.noise_sigma_m,
        'zone_type': ZONE_TYPE_TO_DICT.for_obj(zc.zone_type)(zc.zone_type),
    }


def _zone_config_from_dict(d: dict) -> ZoneConfig:
    zt_d = d['zone_type']
    zone_type = ZONE_TYPE_FROM_DICT.resolve(zt_d['geometry'])(zt_d)
    return ZoneConfig(
        name=d['name'],
        frame=d['frame'],
        color=d['color'],
        expected_intensity=d['expected_intensity'],
        noise_sigma_m=d['noise_sigma_m'],
        zone_type=zone_type,
    )


def _zone_bounds_to_dict(zb) -> dict:
    out = {'name': zb.name, 'zone_config': _zone_config_to_dict(zb.zone_config)}
    out.update(ZONE_BOUNDS_TO_DICT.for_obj(zb)(zb))
    return out


def _zone_bounds_from_dict(d: dict):
    zone_config = _zone_config_from_dict(d['zone_config'])
    return ZONE_BOUNDS_FROM_DICT.resolve(d['geometry'])(d, zone_config)


def _noise_region_to_dict(nr: NoiseRegion) -> dict:
    return {
        'name': nr.name,
        'center': nr.center.tolist(),
        'radius': nr.radius,
        'expected_sigma_m': nr.expected_sigma_m,
        'noise_type': nr.noise_type,
        'z_min': nr.z_min,
        'z_max': nr.z_max,
    }


def _noise_region_from_dict(d: dict) -> NoiseRegion:
    return NoiseRegion(
        name=d['name'],
        center=np.array(d['center'], dtype=np.float64),
        radius=d['radius'],
        expected_sigma_m=d['expected_sigma_m'],
        noise_type=d['noise_type'],
        z_min=d.get('z_min', 0.0),
        z_max=d.get('z_max', 0.0),
    )
