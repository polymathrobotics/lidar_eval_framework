# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from lidar_test_bench_interfaces.msg import NumericalPointCloud, Point4D, ExpectedZone
from lidar_test_bench_interfaces.srv import FilterCloud, GetProfiles, Visualization
from lidar_transforms.tools.marker_builder import MarkerBuilder
from lidar_transforms.tools.profile_builder import BaselineProfiles, FramePose, ProfileBuilder
from lidar_transforms.tools.roi_filter import FilterResult, ROIFilter
from lidar_transforms.tools.roi_loader import ROIConfig, ROILoader
from lidar_transforms.tools.profiles_serializer import profiles_to_json
from lidar_transforms.tools.transform_utils import build_transform_matrix, quat_to_rotation_matrix
from lidar_transforms.tools.zones_utilities import EXPECTED_ZONE_FIELDS


class LidarBaselineNode(Node):
    """Thin ROS2 node — wires together ROIFilter, ProfileBuilder, and MarkerBuilder."""

    def __init__(self) -> None:
        # automatically_declare_parameters_from_overrides=True lets the YAML file
        # auto-declare per-zone padding params (z_padding.<zone>, y_padding.<zone>)
        # since rclpy doesn't accept dict as a parameter type.
        super().__init__('lidar_baseline_node', automatically_declare_parameters_from_overrides=True)

        # Only declare params that don't already come in via the override YAML;
        # auto-declare handles the rest including z_padding.* / y_padding.*.
        for name, default in [
            ('roi_config_path', ''),
            ('cloud_topic', '/rslidar_points'),
            ('markers_topic', '/lidar_baseline/markers'),
            ('filtered_cloud_topic', '/lidar_baseline/filtered_cloud'),
            ('projective_filtered_cloud_topic', '/lidar_baseline/projective_filtered_cloud'),
            ('publish_rate_hz', 1.0),
            ('tf_retry_interval_sec', 1.0),
            ('tf_lookup_timeout_sec', 0.05),
            ('lidar_frame', 'rslidar'),
        ]:
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

        roi_config_path = self.get_parameter('roi_config_path').value
        if not roi_config_path:
            pkg_share = get_package_share_directory('lidar_transforms')
            roi_config_path = str(Path(pkg_share) / 'config' / 'roi.yaml')

        self.get_logger().info(f'Loading ROI config from: {roi_config_path}')
        self._roi_config: ROIConfig = ROILoader().load(roi_config_path)
        self.get_logger().info(
            f'Loaded {len(self._roi_config.zones)} zone(s): '
            f'{[z.name for z in self._roi_config.zones]}'
        )

        self._profiles: Optional[BaselineProfiles] = None
        self._lidar_rotation: Optional[np.ndarray] = None
        self._lidar_position: Optional[np.ndarray] = None
        self._roi_filter = ROIFilter()
        self._marker_builder = MarkerBuilder()

        # YAML's `z_padding: {zone: value}` gets auto-declared as individual
        # `z_padding.<zone>` params. Collect them back into a flat dict.
        self.z_padding_dict = {
            name: param.value for name, param in self.get_parameters_by_prefix('z_padding').items()
        }
        self.y_padding_dict = {
            name: param.value for name, param in self.get_parameters_by_prefix('y_padding').items()
        }


        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        markers_topic = self.get_parameter('markers_topic').value
        filtered_cloud_topic = self.get_parameter('filtered_cloud_topic').value
        projective_filtered_cloud_topic = self.get_parameter('projective_filtered_cloud_topic').value
        self._markers_pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self._filtered_cloud_pub = self.create_publisher(PointCloud2, filtered_cloud_topic, 10)
        self._projective_filtered_cloud_pub = self.create_publisher(PointCloud2, projective_filtered_cloud_topic, 10)

        self._filter_srv = self.create_service(FilterCloud, '/roi_filter', self._handle_filter_cloud)
        self._get_profiles_srv = self.create_service(GetProfiles, '/get_profiles', self._handle_get_profiles)
        self._viz_client = self.create_client(Visualization, '/visualization')

        self._cloud_sub: Optional[object] = None
        self._marker_timer: Optional[object] = None

        retry_interval = self.get_parameter('tf_retry_interval_sec').value
        self._init_timer = self.create_timer(retry_interval, self._try_build_profiles)

    def _lookup_frame_poses(self, timeout_sec: float) -> Optional[dict[str, FramePose]]:
        lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        required_frames = list({z.frame for z in self._roi_config.zones} | {lidar_frame})
        timeout_dur = Duration(seconds=timeout_sec)
        frame_poses: dict[str, FramePose] = {}

        for frame in required_frames:
            try:
                tf_stamped = self._tf_buffer.lookup_transform('map', frame, Time(), timeout_dur)
            except Exception:
                return None
            t = tf_stamped.transform.translation
            q = tf_stamped.transform.rotation
            frame_poses[frame] = FramePose(
                position=np.array([t.x, t.y, t.z]),
                rotation=quat_to_rotation_matrix(q.x, q.y, q.z, q.w),
            )
        return frame_poses

    def _try_build_profiles(self) -> None:
        timeout_sec = self.get_parameter('tf_lookup_timeout_sec').value
        lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        frame_poses = self._lookup_frame_poses(timeout_sec)

        if frame_poses is None:
            self.get_logger().warn('TF lookup failed, retrying…', throttle_duration_sec=5.0)
            return

        try:
            self._profiles = ProfileBuilder().build(self._roi_config, frame_poses, rslidar_frame=lidar_frame)
        except Exception as exc:
            self.get_logger().error(f'ProfileBuilder failed: {exc}')
            return

        self._lidar_rotation = frame_poses[lidar_frame].rotation
        self._lidar_position = frame_poses[lidar_frame].position

        self._init_timer.cancel()
        self.get_logger().info('Profiles built successfully — /get_profiles service is ready')

        publish_rate = self.get_parameter('publish_rate_hz').value
        self._marker_timer = self.create_timer(1.0 / publish_rate, self._publish_markers)

        cloud_topic = self.get_parameter('cloud_topic').value
        self._cloud_sub = self.create_subscription(
            PointCloud2, cloud_topic, self._cloud_callback, 10
        )
        self.get_logger().info(f'Subscribed to cloud topic: {cloud_topic}')

    def _publish_markers(self) -> None:
        if self._profiles is None:
            return
        array = self._marker_builder.build(self._profiles, self.get_clock().now().to_msg())
        self._markers_pub.publish(array)

    def _cloud_callback(self, msg: PointCloud2) -> None:
        if self._profiles is None:
            return

        timeout_sec = self.get_parameter('tf_lookup_timeout_sec').value
        results = self._roi_filter.filter(msg, self._tf_buffer, self._profiles, self.y_padding_dict, self.z_padding_dict, timeout_sec)

        spatial_per_zone = {z: r['spatial_cloud'] for z, r in results.items()}
        projective_per_zone = {z: r['projective_cloud'] for z, r in results.items()}

        spatial_union = self._roi_filter.union_result(spatial_per_zone)
        projective_union = self._roi_filter.union_result(projective_per_zone)

        if not spatial_union.success and not projective_union.success:
            self.get_logger().warn(spatial_union.message, throttle_duration_sec=5.0)
            return

        if spatial_union.success:
            spatial_out = self._roi_filter.to_pointcloud2(spatial_union, msg.header.stamp, use_map_frame=True)
            self._filtered_cloud_pub.publish(spatial_out)

        if projective_union.success:
            projective_out = self._roi_filter.to_pointcloud2(projective_union, msg.header.stamp, use_map_frame=True)
            self._projective_filtered_cloud_pub.publish(projective_out)
            self._push_visualization(projective_union)

    def _push_visualization(self, result: FilterResult) -> None:
        if not self._viz_client.service_is_ready():
            return

        request = Visualization.Request()
        request.viz_msg.roi_cloud = self._build_numerical_cloud(result.filtered_xyz_sensor, result.intensities)
        request.viz_msg.expected_zones = self._build_expected_zones()

        if self._lidar_rotation is not None:
            pitch, roll, yaw = self._rotation_to_euler(self._lidar_rotation)
            request.viz_msg.pitch = pitch
            request.viz_msg.roll = roll
            request.viz_msg.yaw = yaw

        self._viz_client.call_async(request)

    def _build_expected_zones(self) -> list:
        zones = []
        for zb in self._profiles.zone_bounds:
            # The registry returns (lidar-relative) only the ExpectedZone fields
            # this geometry needs; we set just those and leave the rest at
            # defaults. Stays geometry-agnostic — new zone types need no change.
            fields = EXPECTED_ZONE_FIELDS.for_obj(zb)(zb, self._lidar_position)
            z = ExpectedZone()
            z.name = zb.name
            for key, value in fields.items():
                setattr(z, key, value if key == 'geometry' else float(value))
            zones.append(z)
        return zones

    def _build_numerical_cloud(self, xyz: np.ndarray, intensities) -> NumericalPointCloud:
        cloud = NumericalPointCloud()
        for i, (x, y, z) in enumerate(xyz):
            pt = Point4D()
            pt.x, pt.y, pt.z = float(x), float(y), float(z)
            pt.intensity = float(intensities[i]) if intensities is not None else 0.0
            cloud.cloud.append(pt)
        return cloud

    def _rotation_to_euler(self, R: np.ndarray) -> tuple[float, float, float]:
        pitch = float(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        return pitch, roll, yaw

    def _handle_filter_cloud(
        self,
        request: FilterCloud.Request,
        response: FilterCloud.Response,
    ) -> FilterCloud.Response:
        if self._profiles is None:
            response.success = False
            response.message = 'Profiles not yet built — TF frames may not be available yet'
            return response

        timeout_sec = self.get_parameter('tf_lookup_timeout_sec').value
        results = self._roi_filter.filter(
            request.cloud, self._tf_buffer, self._profiles,
            self.y_padding_dict, self.z_padding_dict, timeout_sec,
        )

        # Detect preflight failure: every zone shares the same failure FilterResult instance
        # under both keys (object identity, not just equal contents).
        first_entry = next(iter(results.values()))
        preflight_failed = (
            not first_entry['spatial_cloud'].success
            and not first_entry['projective_cloud'].success
            and first_entry['spatial_cloud'] is first_entry['projective_cloud']
        )

        if preflight_failed:
            response.success = False
            response.message = first_entry['spatial_cloud'].message
            response.zone_names = []
            response.spatial_clouds_per_zone = []
            response.projective_clouds_per_zone = []
            return response

        zone_names: list[str] = []
        spatial_clouds: list = []
        projective_clouds: list = []
        any_zone_fully_ok = False
        fully_ok_count = 0

        for zone_name, per_zone in results.items():
            spatial = per_zone['spatial_cloud']
            projective = per_zone['projective_cloud']

            zone_names.append(zone_name)
            spatial_clouds.append(self._zone_pointcloud(spatial, request.cloud.header.stamp))
            projective_clouds.append(self._zone_pointcloud(projective, request.cloud.header.stamp))

            if spatial.success and projective.success:
                any_zone_fully_ok = True
                fully_ok_count += 1

        response.zone_names = zone_names
        response.spatial_clouds_per_zone = spatial_clouds
        response.projective_clouds_per_zone = projective_clouds
        response.success = any_zone_fully_ok
        response.message = f'{fully_ok_count}/{len(results)} zones have both filters populated'

        return response

    def _zone_pointcloud(self, result: FilterResult, stamp) -> PointCloud2:
        """Pack a per-zone FilterResult into a PointCloud2.

        Returns an empty cloud (zero points, map frame) if the zone's filter failed,
        keeping parallel-array alignment across all zones in the service response.
        """
        if result.success:
            return self._roi_filter.to_pointcloud2(result, stamp, use_map_frame=True)
        empty_header = Header()
        empty_header.stamp = stamp
        empty_header.frame_id = 'map'
        return point_cloud2.create_cloud_xyz32(empty_header, [])

    def _handle_get_profiles(
        self,
        request: GetProfiles.Request,
        response: GetProfiles.Response,
    ) -> GetProfiles.Response:
        if self._profiles is None:
            response.success = False
            response.message = 'Profiles not yet built — TF frames may not be available yet'
            return response

        response.profiles_json = profiles_to_json(self._profiles)
        response.success = True
        response.message = 'OK'
        return response



def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LidarBaselineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
