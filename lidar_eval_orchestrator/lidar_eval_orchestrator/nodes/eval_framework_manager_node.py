#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool
from rclpy.qos import qos_profile_sensor_data

import sys
import os
from pathlib import Path
import traceback


from lidar_metrics.engine import LidarMetricsEngine
from lidar_eval_orchestrator.tools.lidar_processor import LidarProcessor
from lidar_eval_orchestrator.tools.bag_runner import LidarBagPlayer
from std_srvs.srv import Trigger
from lidar_test_bench_interfaces.srv import FilterCloud, GetProfiles
from lidar_zones.zones_api import ZoneEngine
from std_msgs.msg import Int32


class EvalFrameworkManagerNode(Node):
    def __init__(self):
        super().__init__('eval_framework_manager_node')
        self.declare_parameter('input_topic', '/lidar_points')
        self.declare_parameter('bag_timeout', 5)
        self.declare_parameter('lidar_server_topic', '/start_evaluation')
        self.declare_parameter('test_results_dir', "/workspaces/polymath_workspace/src/lidar_testbench/lidar_test_bench_results")
        self.declare_parameter('lidar', "E1R")
        self.declare_parameter('metrics_results_dir', '/workspaces/polymath_workspace/metrics_results')
        self.declare_parameter('environment', '')
        self.declare_parameter('horizontal_resolution_deg', 0.0)
        self.declare_parameter('vertical_resolution_deg', 0.0)


        self.topic_name = self.get_parameter('input_topic').value
        self.bag_timeout = self.get_parameter('bag_timeout').value
        self.lidar_server_topic = self.get_parameter("lidar_server_topic").value
        self.test_results_dir = self.get_parameter('test_results_dir').value
        self.lidar_type = self.get_parameter('lidar').value
        metrics_results_dir = Path(self.get_parameter('metrics_results_dir').value)
        environment = self.get_parameter('environment').value
        if environment:
            metrics_results_dir = metrics_results_dir / environment

        # Initialize the separated logic
        self.processor = LidarProcessor()
        self._zone_engine = ZoneEngine()   # deserializes profiles from /get_profiles
        self.first_pass = False
        self.playing = False

        # need to remove the bottom
        self._servo_angle_pub = self.create_publisher(Int32, '/servo_angle', 10)

        self._report_metrics_client = self.create_client(Trigger, '/report_metrics')

        self.filter_client = self.create_client(FilterCloud, '/roi_filter')
        if not self.filter_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('ROI filter service not available — points will not be filtered')

        self.profiles_client = self.create_client(GetProfiles, '/get_profiles')
        self._profiles_retry_timer = self.create_timer(1.0, self._fetch_profiles)
        # Raw serialized profiles, forwarded to the filter node in each /roi_filter
        # request so it doesn't have to fetch /get_profiles itself.
        self._profiles_json = ''


        self.test_results_folder = os.path.join(self.test_results_dir, self.lidar_type)


        self.engine = LidarMetricsEngine(str(metrics_results_dir), self.get_parameter('horizontal_resolution_deg').value, self.get_parameter('vertical_resolution_deg').value)
        self.bag_player = LidarBagPlayer(self.test_results_folder)
        self.get_logger().info(f"Bag Path: {self.test_results_folder}")
        self.engine.load_registry()

        self.last_lidar_msg_time = self.get_clock().now()
        self.processing_complete = False

        self.watchdog_timer = self.create_timer(1.0, self.check_bag_status)

        self.evaluation_enabled = False
        self.srv = self.create_service(SetBool, self.lidar_server_topic, self.handle_start_evaluation)
        self.get_logger().info(f"SetBool service ready on: {self.lidar_server_topic}")

        self.subscription = self.create_subscription(
            PointCloud2,
            self.topic_name,
            self.lidar_callback,
            qos_profile_sensor_data
        )


    def _fetch_profiles(self) -> None:
        if not self.profiles_client.service_is_ready():
            return
        future = self.profiles_client.call_async(GetProfiles.Request())
        future.add_done_callback(self._on_profiles_response)

    def _on_profiles_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'GetProfiles service call failed: {exc}')
            return

        if not response.success:
            self.get_logger().info(
                f'Profiles not ready yet: {response.message}', throttle_duration_sec=5.0
            )
            return

        self._profiles_retry_timer.cancel()
        self._profiles_json = response.profiles_json
        profiles = self._zone_engine.profiles_from_json(response.profiles_json)
        self.engine.set_base(profiles)
        self.get_logger().info(
            f'BaselineProfiles received: {len(profiles.zone_bounds)} zone(s)'
        )

    def _broadcast_motor_tf(self, angle_deg: int) -> None:
        msg = Int32()
        msg.data = angle_deg
        self._servo_angle_pub.publish(msg)
        self.get_logger().info(f'Servo angle published: {angle_deg} deg')

    def handle_start_evaluation(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.evaluation_enabled = True
            self.processing_complete = False
            self.last_lidar_msg_time = self.get_clock().now()

            self.bag_player.refresh_file_list()
            if not self.bag_player.has_next():
                response.success = False
                response.message = f"No .mcap files found in: {self.test_results_folder}"
                self.get_logger().error(response.message)
                return response

            folder_path, report_stem = self.bag_player.next_bag_report_info()
            self.engine.start_new_test_run(folder_path, report_stem)

            angle = self.bag_player.next_bag_angle()
            self._broadcast_motor_tf(angle if angle is not None else 0)

            started = self.bag_player.play_next_async()
            self.playing = bool(started)

            response.success = True
            response.message = "Evaluation started: playing first bag"
            self.get_logger().info(response.message)
            return response

        self.evaluation_enabled = False
        self.playing = False
        response.success = True
        response.message = "Evaluation disabled"
        self.get_logger().info(response.message)
        return response


    def lidar_callback(self, msg):
        self.last_lidar_msg_time = self.get_clock().now()
        if not self.evaluation_enabled:
            return

        if not self.filter_client.service_is_ready():
            self.get_logger().warn(
                'ROI filter service unavailable — skipping frame; engine needs per-zone data',
                throttle_duration_sec=5.0,
            )
            return

        if not self._profiles_json:
            self.get_logger().warn(
                'Profiles not fetched yet — skipping frame until /get_profiles succeeds',
                throttle_duration_sec=5.0,
            )
            return

        request = FilterCloud.Request()
        request.cloud = msg
        request.profiles_json = self._profiles_json   # filter node uses these, no fetch of its own
        future = self.filter_client.call_async(request)
        future.add_done_callback(self._on_filter_response)

    def _on_filter_response(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'ROI filter service call failed: {exc}')
            return

        # An empty zone_names list is a genuine preflight failure (profiles not
        # built / TF unavailable) — nothing to process, so skip. But success=False
        # *with* zones present just means this scan's zones came back empty; we MUST
        # still feed it to the engine so per-scan metrics (e.g. point counts) count
        # the empty scan. Returning early on `not response.success` made n_scans
        # count only non-empty scans, inflating every "mean per scan" toward total.
        # (Per-scan filter logging — request received, zone counts, point totals —
        # now lives in the pointcloud_filter_node, where the filtering happens.)
        if not response.zone_names:
            return

        # Per-zone clouds from the service (parallel arrays, index i == zone_names[i]).
        # Build two dicts keyed by zone name so the engine can hand each metric
        # the right pre-bucketed clouds without re-deriving per-zone masks.
        spatial_by_zone: dict[str, np.ndarray] = {}
        projective_by_zone: dict[str, np.ndarray] = {}

        for i, zone_name in enumerate(response.zone_names):
            spatial_pts, _ = self.processor.convert_pointcloud2_to_points(response.spatial_clouds_per_zone[i])
            projective_pts, _ = self.processor.convert_pointcloud2_to_points(response.projective_clouds_per_zone[i])
            spatial_by_zone[zone_name] = spatial_pts
            projective_by_zone[zone_name] = projective_pts

        self.engine.run(spatial_by_zone, projective_by_zone)


    def check_bag_status(self):
        if not self.evaluation_enabled or self.processing_complete:
            return

        # Don’t do anything if a bag is still playing
        if self.bag_player.is_playing() or self.bag_player.in_startup_grace():
            return

        # If we thought we were playing, but now the process ended -> bag finished
        if self.playing:
            self.playing = False
            self.get_logger().info("--- Bag playback finished (process ended) ---")
            try:
                self.engine.report()
            except Exception as exc:
                self.get_logger().error(f'engine.report() failed — YAML not written: {exc}\n{traceback.format_exc()}')

            if self._report_metrics_client.service_is_ready():
                self._report_metrics_client.call_async(Trigger.Request())

            if self.bag_player.has_next():
                self.get_logger().info("Starting next bag...")
                folder_path, report_stem = self.bag_player.next_bag_report_info()
                self.engine.start_new_test_run(folder_path, report_stem)
                angle = self.bag_player.next_bag_angle()
                self._broadcast_motor_tf(angle if angle is not None else 0)
                started = self.bag_player.play_next_async()
                self.playing = bool(started)
            else:
                self.get_logger().info("No more bags. Evaluation complete.")
                self.processing_complete = True
                if self._report_metrics_client.service_is_ready():
                    self._report_metrics_client.call_async(Trigger.Request())


def main(args=None):
    rclpy.init(args=args)
    node = EvalFrameworkManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
