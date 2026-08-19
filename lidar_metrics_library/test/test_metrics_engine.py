# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""
High-level tests for LidarMetricsEngine — geometry routing and report shape.

Metric plugins are faked so these stay independent of any individual metric's
math; what's under test is the engine's routing and the report.yaml it produces.
"""

from dataclasses import dataclass, field

from lidar_metrics.engine import LidarMetricsEngine
import numpy as np
import pytest
import yaml


# Bounds stand-ins. The engine derives a zone's geometry from its bounds class
# name ("<Geo>ZoneBounds" -> "<geo>"), so the names here matter.
@dataclass
class PlanarZoneBounds:
    name: str


@dataclass
class CylindricalZoneBounds:
    name: str


@dataclass
class FakeProfiles:
    """Stands in for BaselineProfiles (the engine only needs these two fields)."""

    zone_bounds: list = field(default_factory=list)
    lidar_position: np.ndarray = field(default_factory=lambda: np.zeros(3))


class FakePlugin:
    """Records the per-scan clouds it was fed and returns a canned result."""

    def __init__(self, result, profiles=None):
        self.result = result
        self.profiles = profiles
        self.updates = []
        self.shutdown_called = False

    def update(self, pointcloud_by_zone):
        self.updates.append(dict(pointcloud_by_zone))

    def compute(self):
        return self.result

    def shutdown(self):
        self.shutdown_called = True


@pytest.fixture
def engine(tmp_path):
    """
    Build an engine writing into a temp results root, with profiles wired in directly.

    `_profiles` is set rather than calling set_base() on purpose: set_base() runs
    the params-override pass, which rewrites the package's tracked config.yaml.
    """
    eng = LidarMetricsEngine(str(tmp_path), horizontal_resolution=0.2, vertical_resolution=0.4)
    eng._profiles = FakeProfiles(
        zone_bounds=[PlanarZoneBounds('wall'), CylindricalZoneBounds('post')],
        lidar_position=np.array([0.0, 0.0, 0.5]),
    )
    return eng


def test_load_registry_buckets_shipped_metrics_by_geometry(engine):
    engine.load_registry()

    assert set(engine.plugin_registry) == {'planar', 'cylindrical'}
    for geometry, metrics in engine.plugin_registry.items():
        assert metrics, f'no enabled metrics for {geometry}'
        for info in metrics.values():
            assert info['category'] in ('spatial', 'projective')
            assert info['geometry'] == geometry


def test_run_routes_each_zone_to_its_geometrys_metrics(engine, monkeypatch):
    engine.plugin_registry = {
        'planar': {'PlanarMetric': {'executable': 'p', 'category': 'spatial'}},
        'cylindrical': {'CylMetric': {'executable': 'c', 'category': 'projective'}},
    }
    created = {}

    def fake_create(executable, category, metric_name, geometry,
                    spatial_by_zone, projective_by_zone, profiles=None):
        plugin = FakePlugin({}, profiles=profiles)
        created[metric_name] = plugin
        return plugin

    monkeypatch.setattr(engine, 'create_plugins', fake_create)

    spatial = {'wall': np.zeros((3, 3)), 'post': np.zeros((4, 3))}
    projective = {'wall': np.zeros((2, 3)), 'post': np.zeros((5, 3))}
    engine.run(spatial, projective)

    # Each metric sees only its own geometry's zones, from its own category's dict.
    assert list(created['PlanarMetric'].updates[0]) == ['wall']
    assert list(created['CylMetric'].updates[0]) == ['post']
    assert created['PlanarMetric'].updates[0]['wall'].shape == (3, 3)   # spatial
    assert created['CylMetric'].updates[0]['post'].shape == (5, 3)      # projective

    # Profiles handed to a plugin are restricted to that plugin's geometry.
    assert [zb.name for zb in created['PlanarMetric'].profiles.zone_bounds] == ['wall']
    assert [zb.name for zb in created['CylMetric'].profiles.zone_bounds] == ['post']


def test_run_reuses_plugin_instances_across_scans(engine, monkeypatch):
    engine.plugin_registry = {'planar': {'M': {'executable': 'p', 'category': 'spatial'}}}
    calls = []

    def fake_create(*args, **kwargs):
        calls.append(args)
        return FakePlugin({})

    monkeypatch.setattr(engine, 'create_plugins', fake_create)

    for _ in range(3):
        engine.run({'wall': np.zeros((1, 3))}, {})

    assert len(calls) == 1
    assert len(engine.plugin_instances[('planar', 'M')].updates) == 3


def test_run_without_profiles_is_a_no_op(engine):
    engine._profiles = None
    engine.plugin_registry = {'planar': {'M': {'executable': 'p', 'category': 'spatial'}}}

    engine.run({'wall': np.zeros((1, 3))}, {})

    assert engine.plugin_instances == {}


def test_report_writes_a_three_level_report_yaml(engine, tmp_path):
    # Zone-prefixed keys land under the zone; unmatched keys under __global__.
    prefixed = FakePlugin({'wall_dropout_rate': 0.25, 'wall_dead_cells': 3, 'scan_count': 7})
    zone_keyed = FakePlugin({'post': {'radius_error': np.float64(0.02)}})
    engine.start_new_test_run('E1/at128/base', 'report')
    engine.plugin_instances = {
        ('planar', 'SpatialDropout'): prefixed,
        ('cylindrical', 'RadiusFit'): zone_keyed,
    }

    engine.report()

    report_file = tmp_path / 'E1' / 'at128' / 'base' / 'report.yaml'
    assert report_file.is_file()
    report = yaml.safe_load(report_file.read_text())

    # zone -> Metric -> sub -> value, the shape every downstream consumer walks.
    assert report['wall']['SpatialDropout'] == {'dropout_rate': 0.25, 'dead_cells': 3}
    assert report['post']['RadiusFit'] == {'radius_error': pytest.approx(0.02)}
    assert report['__global__']['SpatialDropout'] == {'scan_count': 7}
    assert report['__global__']['lidar_position'] == [0.0, 0.0, 0.5]

    # compute() is followed by shutdown(), and the engine clears itself for reuse.
    assert prefixed.shutdown_called and zone_keyed.shutdown_called
    assert engine.final_results == {}


def test_report_attributes_underscored_zone_names_to_the_longest_match(engine, tmp_path):
    engine._profiles = FakeProfiles(
        zone_bounds=[PlanarZoneBounds('wall'), PlanarZoneBounds('wall_far_left')],
        lidar_position=np.zeros(3),
    )
    engine.start_new_test_run('E1/at128/base', 'report')
    engine.plugin_instances = {
        ('planar', 'M'): FakePlugin({'wall_far_left_rate': 0.1, 'wall_rate': 0.9}),
    }

    engine.report()

    report = yaml.safe_load((tmp_path / 'E1' / 'at128' / 'base' / 'report.yaml').read_text())
    assert report['wall_far_left']['M'] == {'rate': 0.1}
    assert report['wall']['M'] == {'rate': 0.9}
