# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

import re
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

from lidar_reporting.tools.data_reader import DataReader

_ZONE_COLORS = [
    (0.2, 0.8, 0.2, 0.4),
    (0.9, 0.9, 0.9, 0.4),
    (0.2, 0.5, 0.9, 0.4),
    (0.9, 0.5, 0.2, 0.4),
]


def _quat_from_x_to_normal(normal: np.ndarray) -> tuple:
    """Return (x, y, z, w) quaternion rotating the x-axis onto the given normal."""
    normal = normal / np.linalg.norm(normal)
    from_vec = np.array([1.0, 0.0, 0.0])
    cross = np.cross(from_vec, normal)
    dot = float(np.dot(from_vec, normal))
    cross_norm = float(np.linalg.norm(cross))
    if cross_norm < 1e-6:
        return (0.0, 0.0, 0.0, 1.0) if dot > 0 else (0.0, 1.0, 0.0, 0.0)
    angle = np.arctan2(cross_norm, dot)
    axis = cross / cross_norm
    s = float(np.sin(angle / 2))
    return (axis[0] * s, axis[1] * s, axis[2] * s, float(np.cos(angle / 2)))


class VisualizationNode(Node):

    def __init__(self):
        super().__init__('visualizer_node')

        self.declare_parameter('metrics_results_dir', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('lidar', '')
        self.declare_parameter('environment', '')

        metrics_results_dir = Path(
            self.get_parameter('metrics_results_dir').get_parameter_value().string_value
        )
        self._frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        lidar = self.get_parameter('lidar').get_parameter_value().string_value
        environment = self.get_parameter('environment').get_parameter_value().string_value

        if environment:
            metrics_results_dir = metrics_results_dir / environment
        results_dir = metrics_results_dir / lidar if lidar else metrics_results_dir
        self._data_reader = DataReader(results_dir)

        latching_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(MarkerArray, '/lidar_bench/plane_markers', latching_qos)
        self.create_timer(1.0 / publish_rate, self._publish_markers)
        self.get_logger().info(f'Visualizer reading from: {results_dir}')

    def _publish_markers(self) -> None:
        test_data = self._data_reader.load()
        if not test_data:
            return

        marker_array = MarkerArray()
        marker_id = 0

        for test_case, metrics_data in test_data.items():
            if not isinstance(metrics_data, dict):
                continue
            for metric_name, metric_data in metrics_data.items():
                if not isinstance(metric_data, dict):
                    continue
                viz_data = metric_data.get('visualization', {})
                if not viz_data:
                    continue
                zones = set()
                for key in viz_data:
                    m = re.match(r'^(.+)_plane_center_x$', key)
                    if m:
                        zones.add(m.group(1))
                if not zones:
                    continue
                for color_idx, zone in enumerate(sorted(zones)):
                    marker = self._build_plane_marker(
                        marker_id, viz_data, zone, test_case, metric_name, color_idx
                    )
                    if marker is not None:
                        marker_array.markers.append(marker)
                        marker_id += 1

        self._pub.publish(marker_array)

    def _build_merged_plane_marker(
        self,
        marker_id: int,
        viz_data: dict,
        zones: list[str],
        test_case: str,
        metric_name: str,
    ):
        """Merge all zone planes into a single marker spanning the full surface."""
        centers = []
        normals = []
        y_mins = []
        y_maxs = []
        z_min = None
        z_max = None

        for zone in zones:
            try:
                centers.append(np.array([
                    viz_data[f'{zone}_plane_center_x'],
                    viz_data[f'{zone}_plane_center_y'],
                    viz_data[f'{zone}_plane_center_z'],
                ]))
                normals.append(np.array([
                    viz_data[f'{zone}_plane_normal_x'],
                    viz_data[f'{zone}_plane_normal_y'],
                    viz_data[f'{zone}_plane_normal_z'],
                ]))
                y_mins.append(float(viz_data[f'{zone}_plane_bounds_y_min']))
                y_maxs.append(float(viz_data[f'{zone}_plane_bounds_y_max']))
                z_min = float(viz_data[f'{zone}_plane_bounds_z_min'])
                z_max = float(viz_data[f'{zone}_plane_bounds_z_max'])
            except KeyError:
                continue

        if not centers:
            return None

        center = np.mean(centers, axis=0)
        normal = np.mean(normals, axis=0)
        y_min = min(y_mins)
        y_max = max(y_maxs)

        qx, qy, qz, qw = _quat_from_x_to_normal(normal)
        r, g, b, a = _ZONE_COLORS[0]

        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.ns = f'{metric_name}/{test_case}'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=0, nanosec=0)

        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float((y_min + y_max) / 2.0)
        marker.pose.position.z = float((z_min + z_max) / 2.0)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = 0.005
        marker.scale.y = float(y_max - y_min)
        marker.scale.z = float(z_max - z_min)

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        return marker

    def _build_plane_marker(
        self,
        marker_id: int,
        viz_data: dict,
        zone: str,
        test_case: str,
        metric_name: str,
        color_idx: int,
    ):
        try:
            center = np.array([
                viz_data[f'{zone}_plane_center_x'],
                viz_data[f'{zone}_plane_center_y'],
                viz_data[f'{zone}_plane_center_z'],
            ])
            normal = np.array([
                viz_data[f'{zone}_plane_normal_x'],
                viz_data[f'{zone}_plane_normal_y'],
                viz_data[f'{zone}_plane_normal_z'],
            ])
            y_min = float(viz_data[f'{zone}_plane_bounds_y_min'])
            y_max = float(viz_data[f'{zone}_plane_bounds_y_max'])
            z_min = float(viz_data[f'{zone}_plane_bounds_z_min'])
            z_max = float(viz_data[f'{zone}_plane_bounds_z_max'])
        except KeyError:
            return None

        qx, qy, qz, qw = _quat_from_x_to_normal(normal)
        r, g, b, a = _ZONE_COLORS[color_idx % len(_ZONE_COLORS)]

        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.ns = f'{metric_name}/{test_case}'
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=0, nanosec=0)

        marker.pose.position.x = float(center[0])
        marker.pose.position.y = float(center[1])
        marker.pose.position.z = float(center[2])
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        marker.scale.x = 0.005
        marker.scale.y = float(y_max - y_min)
        marker.scale.z = float(z_max - z_min)

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
