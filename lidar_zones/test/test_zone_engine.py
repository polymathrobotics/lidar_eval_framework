# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""High-level tests for the ZoneEngine — plugin discovery, routing, serialization."""

import numpy as np
import pytest

from lidar_zones.zones_api import ZoneEngine
from lidar_zones.zones_api.profile_types import BaselineProfiles, FramePose, ROIConfig, ZoneConfig
from lidar_zones.zones_api.zone_plugins.cylindrical import CylindricalZonePlugin
from lidar_zones.zones_api.zone_plugins.planar import PlanarZonePlugin


def pose(x=0.0, y=0.0, z=0.0):
    """A FramePose at (x, y, z) with identity rotation."""
    return FramePose(position=np.array([x, y, z], dtype=float), rotation=np.eye(3))


def planar_zone(name='wall', width=2.0):
    """A ZoneConfig carrying a PlanarZoneType."""
    return ZoneConfig(
        name=name, frame=f'{name}_frame', color='white',
        expected_intensity=220.0, noise_sigma_m=0.002,
        zone_type=PlanarZonePlugin.PlanarZoneType(z_bounds=(0.0, 1.0), width=width, y_padding=0.05),
    )


def cylindrical_zone(name='post'):
    """A ZoneConfig carrying a CylindricalZoneType."""
    return ZoneConfig(
        name=name, frame=f'{name}_frame', color='grey',
        expected_intensity=120.0, noise_sigma_m=0.005,
        zone_type=CylindricalZonePlugin.CylindricalZoneType(
            height=2.0, radius=0.5, radius_padding=0.1,
        ),
    )


@pytest.fixture
def engine():
    """A ZoneEngine loaded from the shipped zones_types_registry.yaml."""
    return ZoneEngine()


def test_shipped_registry_exposes_both_geometries(engine):
    assert engine.plugin_for('planar') is PlanarZonePlugin
    assert engine.plugin_for('cylindrical') is CylindricalZonePlugin


def test_plugin_for_unknown_geometry_raises(engine):
    with pytest.raises(KeyError, match='No zone plugin for geometry'):
        engine.plugin_for('trapezoidal')


def test_build_routes_each_zone_type_to_its_plugin(engine):
    planar = engine.build(planar_zone(), pose(x=5.0), np.zeros(3))
    cylindrical = engine.build(cylindrical_zone(), pose(x=4.0, z=1.0), np.zeros(3))

    assert isinstance(planar, PlanarZonePlugin)
    assert isinstance(cylindrical, CylindricalZonePlugin)
    assert engine.geometry_of(planar.bounds) == 'planar'
    assert engine.geometry_of(cylindrical.bounds) == 'cylindrical'


def test_register_zones_is_keyed_by_zone_name(engine):
    roi = ROIConfig(zones=[planar_zone(name='left'), planar_zone(name='right')])
    poses = {'left_frame': pose(x=5.0, y=-1.0), 'right_frame': pose(x=5.0, y=1.0)}

    registered = engine.register_zones(roi, poses, np.zeros(3))

    assert sorted(registered) == ['left', 'right']
    assert registered['left'].bounds.y_min == pytest.approx(-2.0)


def test_wrap_returns_the_plugin_matching_the_bounds_class(engine):
    bounds = engine.build(planar_zone(), pose(x=5.0), np.zeros(3)).bounds

    wrapped = engine.wrap(bounds)

    assert isinstance(wrapped, PlanarZonePlugin)
    assert wrapped.bounds is bounds


def test_profiles_json_round_trip_preserves_both_geometries(engine):
    planar = engine.build(planar_zone(), pose(x=5.0), np.zeros(3))
    cylindrical = engine.build(cylindrical_zone(), pose(x=4.0, z=1.0), np.zeros(3))
    profiles = BaselineProfiles(
        zone_bounds=[planar.bounds, cylindrical.bounds],
        lidar_position=np.array([0.0, 0.0, 0.5]),
    )

    restored = engine.profiles_from_json(engine.profiles_to_json(profiles))

    assert [type(zb) for zb in restored.zone_bounds] == [
        PlanarZonePlugin.PlanarZoneBounds,
        CylindricalZonePlugin.CylindricalZoneBounds,
    ]
    assert [zb.name for zb in restored.zone_bounds] == ['wall', 'post']
    np.testing.assert_allclose(restored.lidar_position, [0.0, 0.0, 0.5])

    planar_restored, cyl_restored = restored.zone_bounds
    assert planar_restored.x_surface == pytest.approx(5.0)
    assert planar_restored.y_padding == pytest.approx(0.05)
    assert cyl_restored.radius_padding == pytest.approx(0.1)
    # zone_config rides along, including its geometry-specific zone_type.
    assert planar_restored.zone_config.zone_type.width == pytest.approx(2.0)
