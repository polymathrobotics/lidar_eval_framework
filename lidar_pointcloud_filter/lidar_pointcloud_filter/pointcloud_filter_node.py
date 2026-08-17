from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

from lidar_test_bench_interfaces.msg import NumericalPointCloud, Point4D, ExpectedZone
from lidar_test_bench_interfaces.srv import FilterCloud, Visualization
from lidar_zones.zones_api import ZoneEngine
from lidar_zones.zones_api.profile_types import BaselineProfiles
from lidar_transforms.transforms import quat_to_rotation_matrix

from lidar_pointcloud_filter.tools.roi_filter import FilterResult, ROIFilter


class PointCloudFilterNode(Node):
    """Filters raw point clouds into per-zone regions of interest.

    Hosts /roi_filter: given a cloud, returns per-zone spatial + projective clouds
    (consumed by the controller to feed the metrics engine). As a side effect of
    each service call it also publishes the unioned filtered clouds on topics and
    pushes a visualization snapshot to /visualization — reusing the *same* filter
    pass, so a scan is filtered exactly once (no separate raw-cloud subscription).
    The live view therefore updates whenever the controller is filtering (i.e.
    during a run), which is the only time there's anything to show.

    Baseline profiles are NOT fetched here — the controller passes them in each
    /roi_filter request (`profiles_json`), so this node never talks to the zones
    node directly. Profiles are cached and only re-deserialized when the string
    changes. The lidar's map-frame rotation (needed for the viz euler angles, which
    the profiles payload doesn't carry) is read from TF whenever profiles change.
    """

    def __init__(self) -> None:
        # automatically_declare_parameters_from_overrides=True lets per-zone padding
        # params (z_padding.<zone>, y_padding.<zone>) auto-declare from the YAML.
        super().__init__('pointcloud_filter_node', automatically_declare_parameters_from_overrides=True)

        for name, default in [
            ('filtered_cloud_topic', '/lidar_baseline/filtered_cloud'),
            ('projective_filtered_cloud_topic', '/lidar_baseline/projective_filtered_cloud'),
            ('lidar_frame', 'rslidar'),
            ('tf_lookup_timeout_sec', 0.05),
        ]:
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

        self._profiles: Optional[BaselineProfiles] = None
        self._profiles_json: str = ''            # cache key — re-deserialize only on change
        self._lidar_position: Optional[np.ndarray] = None
        self._lidar_rotation: Optional[np.ndarray] = None
        self._engine = ZoneEngine()   # routes each zone to its geometry plugin
        self._roi_filter = ROIFilter()

        # YAML's `z_padding: {zone: value}` auto-declares as `z_padding.<zone>`
        # params; collect them back into a flat dict for roi_filter.filter().
        self.z_padding_dict = {
            name: param.value for name, param in self.get_parameters_by_prefix('z_padding').items()
        }
        self.y_padding_dict = {
            name: param.value for name, param in self.get_parameters_by_prefix('y_padding').items()
        }

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        filtered_cloud_topic = self.get_parameter('filtered_cloud_topic').value
        projective_filtered_cloud_topic = self.get_parameter('projective_filtered_cloud_topic').value
        self._filtered_cloud_pub = self.create_publisher(PointCloud2, filtered_cloud_topic, 10)
        self._projective_filtered_cloud_pub = self.create_publisher(PointCloud2, projective_filtered_cloud_topic, 10)

        # The per-scan filter service the controller calls. Publishing + viz happen
        # as a side effect of this handler, so there is no separate cloud subscription.
        # The controller supplies profiles in the request, so no /get_profiles client.
        self._filter_srv = self.create_service(FilterCloud, '/roi_filter', self._handle_filter_cloud)
        self._viz_client = self.create_client(Visualization, '/visualization')

    # ── Profiles (supplied by the controller, cached) ─────────────────────────

    def _ensure_profiles(self, profiles_json: str) -> bool:
        """Make `self._profiles` reflect the given serialized profiles.

        Profiles are static across a run, so we only pay the deserialize + rotation
        TF lookup when the string actually changes; otherwise this is just a string
        compare. Returns False if no usable profiles were supplied (the caller
        should then fail the request).
        """
        if not profiles_json:
            return False
        if profiles_json != self._profiles_json:
            self._profiles = self._engine.profiles_from_json(profiles_json)
            self._profiles_json = profiles_json
            self._lidar_position = self._profiles.lidar_position
            self._lidar_rotation = self._lookup_lidar_rotation()
            self.get_logger().info(
                f'BaselineProfiles updated from request: {len(self._profiles.zone_bounds)} zone(s)'
            )
        return self._profiles is not None

    def _lookup_lidar_rotation(self) -> Optional[np.ndarray]:
        """Map-frame rotation of the lidar, for the viz euler angles.

        Not carried in the profiles payload (only lidar_position is), so read it
        from TF — available by now since the zones node already built profiles
        off the same TF tree.
        """
        lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        timeout_sec = self.get_parameter('tf_lookup_timeout_sec').value
        try:
            tf_stamped = self._tf_buffer.lookup_transform('map', lidar_frame, Time(), Duration(seconds=timeout_sec))
        except Exception as exc:
            self.get_logger().warn(f'Lidar rotation TF lookup failed: {exc}')
            return None
        q = tf_stamped.transform.rotation
        return quat_to_rotation_matrix(q.x, q.y, q.z, q.w)

    # ── Service path (/roi_filter) — the single filter pass ───────────────────

    def _handle_filter_cloud(
        self,
        request: FilterCloud.Request,
        response: FilterCloud.Response,
    ) -> FilterCloud.Response:
        self.get_logger().info('Received cloud for filtering')

        if not self._ensure_profiles(request.profiles_json):
            response.success = False
            response.message = 'No profiles supplied in request (controller has not fetched them yet)'
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
            self.get_logger().warn(f'ROI filter preflight failure: {first_entry["spatial_cloud"].message}')
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

        total_spatial = sum(
            r['spatial_cloud'].filtered_xyz.shape[0]
            for r in results.values() if r['spatial_cloud'].success
        )
        total_projective = sum(
            r['projective_cloud'].filtered_xyz.shape[0]
            for r in results.values() if r['projective_cloud'].success
        )
        self.get_logger().info(f'ROI filter: {response.message}')
        self.get_logger().info(
            f'Spatial points: {total_spatial}, projective points: {total_projective} '
            f'across {len(results)} zones'
        )

        # Reuse the SAME `results` for the live view — publish the unioned filtered
        # clouds and push the viz snapshot. No second filter pass.
        self._publish_and_visualize(results, request.cloud.header.stamp)

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

    # ── Live view (topics + viz), driven by the service handler ───────────────

    def _publish_and_visualize(self, results: dict, stamp) -> None:
        """Publish the unioned filtered clouds and push the viz snapshot.

        Reuses the per-zone `results` already computed in the service handler, so
        the cloud is filtered exactly once per scan.
        """
        spatial_per_zone = {z: r['spatial_cloud'] for z, r in results.items()}
        projective_per_zone = {z: r['projective_cloud'] for z, r in results.items()}

        spatial_union = self._roi_filter.union_result(spatial_per_zone)
        projective_union = self._roi_filter.union_result(projective_per_zone)

        if spatial_union.success:
            spatial_out = self._roi_filter.to_pointcloud2(spatial_union, stamp, use_map_frame=True)
            self._filtered_cloud_pub.publish(spatial_out)

        if projective_union.success:
            projective_out = self._roi_filter.to_pointcloud2(projective_union, stamp, use_map_frame=True)
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
            # Each geometry plugin returns (lidar-relative) only the ExpectedZone
            # fields it needs; the geometry label comes from the engine (plugins
            # don't carry it). We set just those and leave the rest at defaults —
            # stays geometry-agnostic, so new zone types need no change here.
            plugin = self._engine.wrap(zb)
            z = ExpectedZone()
            z.name = zb.name
            z.geometry = self._engine.geometry_of(zb)
            for key, value in plugin.expected_fields(self._lidar_position).items():
                setattr(z, key, float(value))
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

    @staticmethod
    def _rotation_to_euler(R: np.ndarray) -> tuple[float, float, float]:
        pitch = float(np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2)))
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        return pitch, roll, yaw


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PointCloudFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
