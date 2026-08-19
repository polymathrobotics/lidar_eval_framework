# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""High-level tests for ROIFilter — decode, spatial vs projective masking, packing."""

from geometry_msgs.msg import TransformStamped
from lidar_pointcloud_filter.tools.roi_filter import ROIFilter
from lidar_zones.zones_api.profile_types import BaselineProfiles, FramePose, ZoneConfig
from lidar_zones.zones_api.zone_plugins.planar import PlanarZonePlugin
import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class StubTfBuffer:
    """Minimal tf2 Buffer stand-in: returns a fixed transform, or raises."""

    def __init__(self, transform=None, error=None):
        self._transform = transform
        self._error = error

    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        if self._error is not None:
            raise self._error
        return self._transform


def identity_transform():
    """Build a sensor->map TransformStamped with no translation or rotation."""
    tf = TransformStamped()
    tf.transform.rotation.w = 1.0
    return tf


def make_cloud(points, with_intensity=True):
    """Pack (x, y, z[, intensity]) tuples into a PointCloud2 in the 'rslidar' frame."""
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    if with_intensity:
        fields.append(
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        )
    header = Header()
    header.frame_id = 'rslidar'
    return point_cloud2.create_cloud(header, fields, [list(p) for p in points])


@pytest.fixture
def profiles():
    """One planar zone: surface at x=5, y in [-1, 1], z in [0, 1]. Lidar at the origin."""
    zone_cfg = ZoneConfig(
        name='wall', frame='wall_frame', color='white',
        expected_intensity=220.0, noise_sigma_m=0.002,
        zone_type=PlanarZonePlugin.PlanarZoneType(z_bounds=(0.0, 1.0), width=2.0),
    )
    pose = FramePose(position=np.array([5.0, 0.0, 0.0]), rotation=np.eye(3))
    bounds = PlanarZonePlugin.build(zone_cfg, pose, np.zeros(3)).bounds
    return BaselineProfiles(zone_bounds=[bounds], lidar_position=np.zeros(3))


def test_filter_splits_spatial_and_projective_per_zone(profiles):
    # The spatial box is a 20cm slab around x=5; the projective cone reaches to
    # infinity, so the near point at x=2 lands in one and not the other.
    cloud = make_cloud([
        (5.0, 0.0, 0.5, 10.0),    # in the slab and in the cone
        (5.0, 0.5, 0.2, 20.0),    # in the slab and in the cone
        (2.0, 0.0, 0.1, 30.0),    # in the cone only (too near for the slab)
        (10.0, 8.0, 0.5, 40.0),   # outside both (way off-bearing)
        (0.0, 0.0, 0.0, 50.0),    # invalid sensor return, dropped before masking
    ])

    result = ROIFilter().filter(cloud, StubTfBuffer(identity_transform()), profiles, {}, {})

    spatial = result['wall']['spatial_cloud']
    projective = result['wall']['projective_cloud']
    assert spatial.success and spatial.filtered_xyz.shape[0] == 2
    assert projective.success and projective.filtered_xyz.shape[0] == 3
    assert spatial.has_intensity
    np.testing.assert_allclose(sorted(spatial.intensities), [10.0, 20.0])


def test_projective_padding_tightens_the_cone(profiles):
    # Point sits near the zone's y edge: inside the raw cone, outside a padded one.
    cloud = make_cloud([(5.0, 0.9, 0.5, 1.0)])
    roi_filter = ROIFilter()
    tf_buffer = StubTfBuffer(identity_transform())

    unpadded = roi_filter.filter(cloud, tf_buffer, profiles, {}, {})
    padded = roi_filter.filter(cloud, tf_buffer, profiles, {'wall': 0.5}, {})

    assert unpadded['wall']['projective_cloud'].success
    assert not padded['wall']['projective_cloud'].success


def test_empty_zone_is_reported_as_an_unsuccessful_result(profiles):
    cloud = make_cloud([(1.0, 0.0, 0.0, 1.0)])   # nowhere near the zone

    result = ROIFilter().filter(cloud, StubTfBuffer(identity_transform()), profiles, {}, {})

    spatial = result['wall']['spatial_cloud']
    assert not spatial.success
    assert spatial.filtered_xyz is None
    assert 'No points in spatial bbox' in spatial.message


def test_missing_xyz_fields_fail_every_zone_uniformly(profiles):
    cloud = PointCloud2()
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    ]

    result = ROIFilter().filter(cloud, StubTfBuffer(identity_transform()), profiles, {}, {})

    # Callers iterate the same shape on the failure path — both keys are present.
    for key in ('spatial_cloud', 'projective_cloud'):
        assert not result['wall'][key].success
        assert 'missing XYZ' in result['wall'][key].message


def test_tf_lookup_failure_fails_every_zone_uniformly(profiles):
    cloud = make_cloud([(5.0, 0.0, 0.5, 1.0)])

    result = ROIFilter().filter(
        cloud, StubTfBuffer(error=LookupError('no such frame')), profiles, {}, {}
    )

    assert not result['wall']['spatial_cloud'].success
    assert 'TF lookup failed' in result['wall']['spatial_cloud'].message


def test_union_result_aggregates_only_successful_zones(profiles):
    roi_filter = ROIFilter()
    cloud = make_cloud([(5.0, 0.0, 0.5, 1.0), (5.0, 0.5, 0.2, 2.0)])
    per_zone = roi_filter.filter(cloud, StubTfBuffer(identity_transform()), profiles, {}, {})
    spatial_only = {'wall': per_zone['wall']['spatial_cloud']}

    union = roi_filter.union_result(spatial_only)

    assert union.success and union.filtered_xyz.shape == (2, 3)

    empty = roi_filter.filter(
        make_cloud([(1.0, 0.0, 0.0, 1.0)]), StubTfBuffer(identity_transform()),
        profiles, {}, {},
    )
    no_points = roi_filter.union_result({'wall': empty['wall']['spatial_cloud']})
    assert not no_points.success
    assert no_points.filtered_xyz is None


def test_to_pointcloud2_round_trips_the_filtered_points(profiles):
    from builtin_interfaces.msg import Time

    roi_filter = ROIFilter()
    cloud = make_cloud([(5.0, 0.0, 0.5, 42.0)])
    result = roi_filter.filter(
        cloud, StubTfBuffer(identity_transform()), profiles, {}, {}
    )['wall']['spatial_cloud']

    packed = roi_filter.to_pointcloud2(result, Time(), frame_id='rslidar', use_map_frame=True)

    assert packed.header.frame_id == 'map'
    read_back = point_cloud2.read_points(packed, field_names=['x', 'y', 'z', 'intensity'])
    assert len(read_back) == 1
    assert (read_back['x'][0], read_back['z'][0]) == pytest.approx((5.0, 0.5))
    assert read_back['intensity'][0] == pytest.approx(42.0)
