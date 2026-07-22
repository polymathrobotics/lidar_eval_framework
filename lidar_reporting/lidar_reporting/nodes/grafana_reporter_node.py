# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

import base64
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_srvs.srv import Trigger
from lidar_test_bench_interfaces.srv import Visualization
from tf2_ros import Buffer, TransformListener

from lidar_reporting.tools.lidar_database_handler import LidarDatabaseHandler
from lidar_reporting.tools.data_reader import DataReader


class GrafanaReporterNode(Node):
    def __init__(self):
        super().__init__('grafana_reporter_node')

        self.declare_parameter('metrics_results_dir', '')
        self.declare_parameter('lidar_frame', 'rslidar')
        self.declare_parameter('lidar_cost', 0.0)
        self.declare_parameter('lidar_horizontal_fov_deg', 0.0)
        self.declare_parameter('lidar_vertical_fov_deg', 0.0)
        self.declare_parameter('bag_directory', '')

        metrics_results_dir = Path(self.get_parameter('metrics_results_dir').get_parameter_value().string_value)
        bag_directory = Path(self.get_parameter('bag_directory').get_parameter_value().string_value)
        lidar_metadata = {
            'lidar_cost': self.get_parameter('lidar_cost').get_parameter_value().double_value,
            'lidar_horizontal_fov_deg': self.get_parameter('lidar_horizontal_fov_deg').get_parameter_value().double_value,
            'lidar_vertical_fov_deg': self.get_parameter('lidar_vertical_fov_deg').get_parameter_value().double_value,
        }

        self.data_reader = DataReader(metrics_results_dir, bag_directory)
        self.test_data = {}
        self._current_location: tuple[str, str, str] | None = None
        self._pending_viz_blocks: list | None = None
        self._viz_snapshots: dict[tuple[str, str, str], list] = {}
        self._prev_case_count: int = 0

        self.db_handler = LidarDatabaseHandler(lidar_metadata=lidar_metadata)
        self.get_logger().info('Authenticating with Google Drive via 1Password — follow the prompts in this terminal...')
        try:
            self.db_handler.authenticate()
            self.get_logger().info('Google Drive authentication successful')
        except Exception as e:
            self.get_logger().error(
                f'Google Drive authentication failed: {e}. Drive sync and visualization push will be disabled.'
            )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_service(Trigger, '/report_metrics', self._on_report_trigger)
        self.create_service(Visualization, '/visualization', self._on_visualization_request)
        self.get_logger().info('Report trigger service ready on: /report_metrics')

        self.test_data = self.data_reader.load()
        if self.test_data:
            self._current_location = self._first_location()
            self.get_logger().info(f'Auto-loaded {self._count_cases()} existing test case(s) on startup')

    def _first_location(self) -> tuple[str, str, str] | None:
        for env, lidar_data in self.test_data.items():
            for lidar, case_data in lidar_data.items():
                for case in case_data:
                    return (env, lidar, case)
        return None

    def _count_cases(self) -> int:
        return sum(len(case_data) for lidar_data in self.test_data.values() for case_data in lidar_data.values())

    def _on_report_trigger(self, _request, response):
        self.test_data = self.data_reader.load()
        current_count = self._count_cases()
        self._current_location = self.data_reader.most_recent_case()
        self.get_logger().info(f'Loaded {current_count} test case(s)')

        if current_count > self._prev_case_count:
            # New case just written — snapshot its viz data for later push
            if self._pending_viz_blocks is not None and self._current_location:
                all_blocks = (
                    self._pending_viz_blocks
                    + self._fitted_plane_blocks()
                    + self._dead_cell_blocks()
                    + self._worst_point_blocks()
                )
                self._viz_snapshots[self._current_location] = all_blocks
                self._pending_viz_blocks = None
                self.get_logger().info(
                    f'Viz snapshot saved for: {"/".join(self._current_location)}'
                )
            self._prev_case_count = current_count
            response.success = True
            response.message = f'Snapshotted case {current_count}'
            return response

        # Case count unchanged — final signal, push everything to Google Drive
        self.get_logger().info('Final push: syncing results and all viz snapshots to Google Drive')
        try:
            rosbags = self.data_reader.load_rosbags()
            self.db_handler.sync(self.test_data, rosbags)
            self.get_logger().info('Google Drive sync complete')
        except Exception as e:
            self.get_logger().error(f'Google Drive sync failed: {e}')

        if self.db_handler.available:
            for location, blocks in self._viz_snapshots.items():
                env, lidar, case = location
                try:
                    self.db_handler.push_visualization(env, lidar, case, blocks)
                    self.get_logger().info(f'Google Drive viz pushed to: {env}/{lidar}/{case}')
                except Exception as e:
                    self.get_logger().error(f'Google Drive viz push failed for {env}/{lidar}/{case}: {e}')

        self._viz_snapshots.clear()
        self._prev_case_count = 0
        response.success = True
        response.message = f'Reported {current_count} test case(s)'
        return response

    def _on_visualization_request(self, request, response):
        msg = request.viz_msg
        self._pending_viz_blocks = [
            self._viz_heading('Orientation'),
            self._viz_bullet(f'pitch: {msg.pitch:.6g}'),
            self._viz_bullet(f'roll: {msg.roll:.6g}'),
            self._viz_bullet(f'yaw: {msg.yaw:.6g}'),
            *self._expected_zone_blocks(msg.expected_zones),
            *self._cloud_blocks('roi_cloud', msg.roi_cloud),
        ]
        response.success = True
        response.message = 'Viz data buffered — will push when /report_metrics is called'
        return response

    def _get_lidar_pose(self):
        lidar_frame = self.get_parameter('lidar_frame').get_parameter_value().string_value
        try:
            tf_stamped = self._tf_buffer.lookup_transform('map', lidar_frame, Time(), Duration(seconds=0.1))
            q = tf_stamped.transform.rotation
            tr = tf_stamped.transform.translation
            x, y, z, w = q.x, q.y, q.z, q.w
            R = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)    ],
                [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)    ],
                [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
            ])
            t = np.array([tr.x, tr.y, tr.z])
            return R, t
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed for lidar pose: {e}')
            return None, None

    def _transform_fitted_viz(self, viz: dict, R: np.ndarray, t: np.ndarray) -> dict:
        result = dict(viz)
        zones = [k.replace('_plane_center_x', '') for k in viz if k.endswith('_plane_center_x')]
        for zone in zones:
            cx = float(viz.get(f'{zone}_plane_center_x', 0.0))
            cy = float(viz.get(f'{zone}_plane_center_y', 0.0))
            cz = float(viz.get(f'{zone}_plane_center_z', 0.0))
            y_min = float(viz.get(f'{zone}_plane_bounds_y_min', cy - 0.5))
            y_max = float(viz.get(f'{zone}_plane_bounds_y_max', cy + 0.5))
            z_min = float(viz.get(f'{zone}_plane_bounds_z_min', cz - 0.5))
            z_max = float(viz.get(f'{zone}_plane_bounds_z_max', cz + 0.5))
            center_disp = np.array([cx, cy, cz]) - t
            result[f'{zone}_plane_center_x'] = float(center_disp[0])
            result[f'{zone}_plane_center_y'] = float(center_disp[1])
            result[f'{zone}_plane_center_z'] = float(center_disp[2])
            corners = np.array([
                [cx, y_min, z_min],
                [cx, y_max, z_min],
                [cx, y_min, z_max],
                [cx, y_max, z_max],
            ])
            corners_disp = corners - t
            result[f'{zone}_plane_bounds_y_min'] = float(corners_disp[:, 1].min())
            result[f'{zone}_plane_bounds_y_max'] = float(corners_disp[:, 1].max())
            result[f'{zone}_plane_bounds_z_min'] = float(corners_disp[:, 2].min())
            result[f'{zone}_plane_bounds_z_max'] = float(corners_disp[:, 2].max())

        expected_zones = [k.replace('_expected_x', '') for k in viz if k.endswith('_expected_x')]
        for zone in expected_zones:
            for axis, offset in (('x', t[0]), ('y_min', t[1]), ('y_max', t[1]),
                                 ('z_min', t[2]), ('z_max', t[2])):
                ekey = f'{zone}_expected_{axis}'
                if ekey in result:
                    result[ekey] = float(viz[ekey]) - float(offset)
        return result

    def _fitted_plane_blocks(self) -> list:
        if not self._current_location:
            return []
        env, lidar, case = self._current_location
        R, t = self._get_lidar_pose()
        case_data = self.test_data.get(env, {}).get(lidar, {}).get(case, {})
        blocks = []

        for zone, metrics in case_data.items():
            if not isinstance(metrics, dict):
                continue
            for values in metrics.values():
                if not isinstance(values, dict):
                    continue
                viz = values.get('visualization')
                if not viz:
                    continue
                prefixed = {f'{zone}_{key}': val for key, val in viz.items()}
                if R is not None and t is not None:
                    prefixed = self._transform_fitted_viz(prefixed, R, t)
                blocks.append(self._viz_heading(f'{zone} · FittedPlane'))
                for key, val in prefixed.items():
                    if isinstance(val, (int, float)):
                        blocks.append(self._viz_bullet(f'{key}: {val:.6g}'))
        return blocks

    def _dead_cell_blocks(self) -> list:
        if not self._current_location:
            return []
        env, lidar, case = self._current_location
        _, t = self._get_lidar_pose()
        case_data = self.test_data.get(env, {}).get(lidar, {}).get(case, {})

        # Report nests zone -> SpatialDropout -> {dead_cell_<n>_y_m/_z_m, ...}.
        # The engine strips the zone prefix from the sub-keys, so match on the
        # bare 'dead_cell_' and re-attach the zone when emitting.
        cell_size = None
        bullets = []
        for zone, metrics in case_data.items():
            if not isinstance(metrics, dict):
                continue
            sd = metrics.get('SpatialDropout')
            if not isinstance(sd, dict):
                continue
            for key, val in sd.items():
                if key == 'dead_cell_size_m':
                    cell_size = val
                    continue
                if not key.startswith('dead_cell_'):
                    continue
                if t is not None and isinstance(val, (int, float)):
                    if key.endswith('_y_m'):
                        val -= t[1]
                    elif key.endswith('_z_m'):
                        val -= t[2]
                bullets.append(self._viz_bullet(f'{zone}_{key}: {val:.6g}'))

        if not bullets and cell_size is None:
            return []
        blocks = [self._viz_heading('SpatialDropout · DeadCells')]
        if cell_size is not None:
            blocks.append(self._viz_bullet(f'dead_cell_size_m: {cell_size:.6g}'))
        blocks.extend(bullets)
        return blocks

    def _worst_point_blocks(self) -> list:
        if not self._current_location:
            return []
        env, lidar, case = self._current_location
        _, t = self._get_lidar_pose()
        case_data = self.test_data.get(env, {}).get(lidar, {}).get(case, {})

        # ZoneSurfaceDepthError / RangeDistributionHealth each report their top-K
        # worst points as worst_point_<n>_{x,y,z} in map frame. Translate into the
        # lidar-relative frame (subtract t — same as fitted planes / dead cells, no
        # rotation) so they overlay the rendered cloud, and tag by metric.
        metric_tags = {'RangeDistributionHealth': 'range', 'ZoneSurfaceDepthError': 'depth'}
        bullets = []
        for zone, metrics in case_data.items():
            if not isinstance(metrics, dict):
                continue
            for metric_name, tag in metric_tags.items():
                md = metrics.get(metric_name)
                if not isinstance(md, dict):
                    continue
                n = 0
                while f'worst_point_{n}_x' in md:
                    x = float(md[f'worst_point_{n}_x'])
                    y = float(md[f'worst_point_{n}_y'])
                    z = float(md[f'worst_point_{n}_z'])
                    if t is not None:
                        x -= float(t[0])
                        y -= float(t[1])
                        z -= float(t[2])
                    base = f'{zone}_{tag}_{n}'
                    bullets.append(self._viz_bullet(f'{base}_x: {x:.6g}'))
                    bullets.append(self._viz_bullet(f'{base}_y: {y:.6g}'))
                    bullets.append(self._viz_bullet(f'{base}_z: {z:.6g}'))
                    n += 1
        if not bullets:
            return []
        return [self._viz_heading('WorstPoints')] + bullets

    def _expected_zone_blocks(self, expected_zones) -> list:
        if not expected_zones:
            return []
        blocks = [self._viz_heading('ExpectedZones · FittedPlane')]
        for z in expected_zones:
            blocks.append(self._viz_bullet(f'{z.name}_expected_x: {z.x:.6g}'))
            blocks.append(self._viz_bullet(f'{z.name}_expected_y_min: {z.y_min:.6g}'))
            blocks.append(self._viz_bullet(f'{z.name}_expected_y_max: {z.y_max:.6g}'))
            blocks.append(self._viz_bullet(f'{z.name}_expected_z_min: {z.z_min:.6g}'))
            blocks.append(self._viz_bullet(f'{z.name}_expected_z_max: {z.z_max:.6g}'))
        return blocks

    def _cloud_blocks(self, name, cloud) -> list:
        arr = np.array(
            [[p.x, p.y, p.z, p.intensity] for p in cloud.cloud], dtype=np.float32
        )
        encoded = base64.b64encode(arr.tobytes()).decode('ascii')
        blocks = [self._viz_heading(f'{name} · base64 float32 · shape {arr.shape[0]}x4')]
        for i in range(0, len(encoded), 2000):
            blocks.append({
                'object': 'block', 'type': 'paragraph',
                'paragraph': {'rich_text': [{'type': 'text', 'text': {'content': encoded[i:i + 2000]}}]},
            })
        return blocks

    def _viz_heading(self, text, level=2) -> dict:
        t = f'heading_{level}'
        return {'object': 'block', 'type': t, t: {'rich_text': [{'type': 'text', 'text': {'content': text}}]}}

    def _viz_bullet(self, text) -> dict:
        return {
            'object': 'block', 'type': 'bulleted_list_item',
            'bulleted_list_item': {'rich_text': [{'type': 'text', 'text': {'content': text}}]},
        }


def main(args=None):
    rclpy.init(args=args)
    node = GrafanaReporterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
