# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from lidar_test_bench_interfaces.srv import GetProfiles
from lidar_zones.zones_api import ZoneEngine
from lidar_zones.zones_api.profile_types import BaselineProfiles, FramePose
from lidar_transforms.transforms import quat_to_rotation_matrix
from lidar_zones.zones_builder_tools.marker_builder import MarkerBuilder
from lidar_zones.zones_builder_tools.profile_builder import ProfileBuilder
from lidar_zones.zones_builder_tools.roi_loader import ROIConfig, ROILoader


class ZonesOrchestratorNode(Node):
    """Builds per-zone baseline profiles at startup, publishes their RViz markers,
    and serves them over /get_profiles.

    Profiles are built from the TF tree (published from the bench URDF by
    robot_state_publisher) plus roi.yaml — not by parsing the URDF directly. TF is
    retried on a timer until available, then the profiles are frozen. /get_profiles
    is a getter that returns the already-built profiles; nothing triggers the build
    on request. Point-cloud filtering and visualization live in the pointcloud
    filter node.
    """

    def __init__(self) -> None:
        # automatically_declare_parameters_from_overrides=True lets params supplied
        # via the launch/override YAML auto-declare without an explicit entry here.
        super().__init__('zones_orchestrator_node', automatically_declare_parameters_from_overrides=True)

        for name, default in [
            ('roi_config_path', ''),
            ('markers_topic', '/lidar_baseline/markers'),
            ('publish_rate_hz', 1.0),
            ('tf_retry_interval_sec', 1.0),
            ('tf_lookup_timeout_sec', 0.05),
            ('lidar_frame', 'rslidar'),
        ]:
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

        roi_config_path = self.get_parameter('roi_config_path').value
        if not roi_config_path:
            pkg_share = get_package_share_directory('lidar_zones')
            roi_config_path = str(Path(pkg_share) / 'config' / 'roi.yaml')

        self.get_logger().info(f'Loading ROI config from: {roi_config_path}')
        self._roi_config: ROIConfig = ROILoader().load(roi_config_path)
        self.get_logger().info(
            f'Loaded {len(self._roi_config.zones)} zone(s): '
            f'{[z.name for z in self._roi_config.zones]}'
        )

        self._profiles: Optional[BaselineProfiles] = None
        self._engine = ZoneEngine()   # serializes profiles for /get_profiles
        self._marker_builder = MarkerBuilder()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        markers_topic = self.get_parameter('markers_topic').value
        self._markers_pub = self.create_publisher(MarkerArray, markers_topic, 10)

        self._get_profiles_srv = self.create_service(GetProfiles, '/get_profiles', self._handle_get_profiles)

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
            self._profiles = ProfileBuilder().build(self._roi_config, frame_poses, lidar_frame=lidar_frame)
        except Exception as exc:
            self.get_logger().error(f'ProfileBuilder failed: {exc}')
            return

        self._init_timer.cancel()
        self.get_logger().info('Profiles built successfully — /get_profiles service is ready')

        publish_rate = self.get_parameter('publish_rate_hz').value
        self._marker_timer = self.create_timer(1.0 / publish_rate, self._publish_markers)

    def _publish_markers(self) -> None:
        if self._profiles is None:
            return
        array = self._marker_builder.build(self._profiles, self.get_clock().now().to_msg())
        self._markers_pub.publish(array)

    def _handle_get_profiles(
        self,
        request: GetProfiles.Request,
        response: GetProfiles.Response,
    ) -> GetProfiles.Response:
        if self._profiles is None:
            response.success = False
            response.message = 'Profiles not yet built — TF frames may not be available yet'
            return response

        response.profiles_json = self._engine.profiles_to_json(self._profiles)
        response.success = True
        response.message = 'OK'
        return response


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ZonesOrchestratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
