# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

import math
import os
from pathlib import Path

import yaml

from lidar_zones.zones_api.zone_engine import ZoneEngine
from lidar_zones.zones_gen.urdf_generator import URDFGenerator
from polysetup.polysetup_utils import PolysetupUtils


class ConfigureNewRun:

    def __init__(self, lidar_config: dict, environment_config: dict,
                 src_dir: Path | None = None):
        self.lidar_config = lidar_config
        self.environment_config = environment_config
        self.lidar_params = lidar_config.get('lidar', {})
        self.utility_manager = PolysetupUtils()
        self._zone_engine = ZoneEngine()

        # Build every downstream path from the workspace src dir at runtime, so this works no
        # matter where polysetup is invoked from. Defaults to the workspace root (three levels up
        # from this file: polysetup/polysetup/configure_new_run.py) when no src dir is passed.
        src_dir = Path(src_dir) if src_dir is not None else Path(__file__).resolve().parents[2]
        self._src_dir = src_dir
        self._urdf_output_path = src_dir / 'lidar_transforms' / 'urdf' / 'lidar_bench.urdf'
        self._eval_framework_manager_yaml = src_dir / 'lidar_eval_orchestrator' / 'config' / 'eval_framework_manager.yaml'
        self._zones_config_yaml = src_dir / 'lidar_zones' / 'config' / 'zones_orchestrator.yaml'
        self._pointcloud_filter_yaml = src_dir / 'lidar_pointcloud_filter' / 'config' / 'pointcloud_filter.yaml'
        self._lidar_reporting_config_yaml = src_dir / 'lidar_reporting' / 'config' / 'config.yaml'
        self._roi_yaml_path = src_dir / 'lidar_zones' / 'config' / 'roi.yaml'
        self._color_map_path = src_dir / 'lidar_zones' / 'lidar_zones' / 'zones_gen' / 'config' / 'color_map.yaml'
        self._automation_manager_yaml = src_dir / 'lidar_automation_manager' / 'config' / 'automation_manager.yaml'
        self._bringup_launch_yaml = src_dir / 'lidar_test_bench_bringup' / 'launch' / 'lidar_test_bench_launch.yaml'
        self._metrics_results_dir = str(src_dir / 'results_locations' / 'metrics_results')
        self._base_data_dir = str(src_dir / 'results_locations' / 'rosbags')

        with self._color_map_path.open() as f:
            self._color_map = yaml.safe_load(f)['color_map']

        roi_zones = self.update_roi_regions()
        with self._roi_yaml_path.open('w') as f:
            yaml.safe_dump({'zones': roi_zones}, f, sort_keys=False, default_flow_style=False)


    def create_base_launch_file(self) -> None:
        """Recreate the bringup launch file with the evaluation node set.

        The recording chain (arduino_eth_bridge, bag_recorder, the automation manager) is never
        written here — polysetup-bag-recording adds or restores those entries on demand.
        """

        launch_entries = [
            {
                'node': {
                    'pkg': 'lidar_eval_orchestrator',
                    'exec': 'eval_framework_manager_node',
                    'name': 'eval_framework_manager_node',
                    'output': 'log',
                    'param': [{'from': '$(find-pkg-share lidar_eval_orchestrator)/config/eval_framework_manager.yaml'}],
                }
            },
            {
                'include': {
                    'file': '$(find-pkg-share lidar_transforms)/launch/robot_description.launch.py',
                }
            },
            {
                'node': {
                    'pkg': 'lidar_transforms',
                    'exec': 'lidar_bench_tf_broadcaster',
                    'name': 'lidar_bench_tf_broadcaster',
                    'output': 'log',
                }
            },
            {
                'node': {
                    'pkg': 'lidar_reporting',
                    'exec': 'metrics_reporting_node',
                    'name': 'metrics_reporting_node',
                    'output': 'screen',
                    'param': [{'from': '$(find-pkg-share lidar_reporting)/config/config.yaml'}],
                }
            },
            {
                'node': {
                    'pkg': 'lidar_reporting',
                    'exec': 'visualizer_node',
                    'name': 'visualizer_node',
                    'output': 'log',
                    'param': [{'from': '$(find-pkg-share lidar_reporting)/config/config.yaml'}],
                }
            },
            {
                'node': {
                    'pkg': 'lidar_zones',
                    'exec': 'zones_orchestrator_node',
                    'name': 'zones_orchestrator_node',
                    'output': 'log',
                    'param': [{'from': '$(find-pkg-share lidar_zones)/config/zones_orchestrator.yaml'}],
                }
            },
            {
                'node': {
                    'pkg': 'lidar_pointcloud_filter',
                    'exec': 'pointcloud_filter_node',
                    'name': 'pointcloud_filter_node',
                    'output': 'log',
                    'param': [{'from': '$(find-pkg-share lidar_pointcloud_filter)/config/pointcloud_filter.yaml'}],
                }
            },
            {
                'node': {
                    'pkg': 'arduino_eth_bridge',
                    'exec': 'arduino_eth_bridge_node',
                    'name': 'arduino_eth_bridge_node',
                    'output': 'screen',
                    'param': [{'from': '$(find-pkg-share arduino_eth_bridge)/config/arduino_eth_bridge.yaml'}],
                }
            },
            {
                'node': {
                    'pkg': 'lidar_automation_manager',
                    'exec': 'automation_manager',
                    'name': 'lidar_automation_manager',
                    'output': 'screen',
                    'param': [{'from': '$(find-pkg-share lidar_automation_manager)/config/automation_manager.yaml'}],
                }
            },
            {
                'include': {
                    'file': '$(find-pkg-share foxglove_bridge)/launch/foxglove_bridge_launch.xml',
                }
            },
        ]

        launch = {'launch': launch_entries}
        with self._bringup_launch_yaml.open('w') as f:
            yaml.safe_dump(launch, f, sort_keys=False, default_flow_style=False)
        print(f'  lidar_test_bench_launch.yaml  recreated with base node set')


    def configure(self) -> None:
        print(f'Configuring run: lidar={self.lidar_params.get("frame")}, environment={self.environment_config.get("name", "unknown")}')
        self._build_urdf()
        self._update_lidar_test_bench_yaml()
        self._update_zones_config_yaml()
        self._update_pointcloud_filter_yaml()
        self._update_lidar_reporting_config_yaml()
        self._update_automation_manager_yaml()
        self._update_bringup_launch_yaml()
        print('Done.')

    def _build_urdf(self) -> None:
        builder = URDFGenerator(
            environment_config_dict=self.environment_config,
            lidar_config_dict=self.lidar_config,
        )
        builder.build_harness(self._urdf_output_path)
        print(f'  lidar_bench.urdf       written to {self._urdf_output_path}')

    def _update_lidar_test_bench_yaml(self) -> None:


        input_topic = self.lidar_params.get('point_cloud_topic')
        lidar_folder = self.lidar_params.get('folder')
        environment = self.environment_config.get('name', '')
        bag_recorder_directory = self.environment_config.get('bag_recorder_directory')



        vertical_resolution = self.lidar_params.get('vertical_resolution_deg')
        horizontal_resolution = self.lidar_params.get('horizontal_resolution_deg')
        if vertical_resolution is not None:
            vertical_resolution = float(vertical_resolution)
        if horizontal_resolution is not None:
            horizontal_resolution = float(horizontal_resolution)

        with self._eval_framework_manager_yaml.open() as f:
            config = yaml.safe_load(f)
        params = config['eval_framework_manager_node']['ros__parameters']
        params['input_topic'] = input_topic
        params['lidar'] = lidar_folder
        params['environment'] = environment
        params['test_results_dir'] = self._base_data_dir + "/" + bag_recorder_directory
        params['metrics_results_dir'] = self._metrics_results_dir
        params['horizontal_resolution_deg'] = horizontal_resolution
        params['vertical_resolution_deg'] = vertical_resolution



        with self._eval_framework_manager_yaml.open('w') as f:
            yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False)
        print(f'  eval_framework_manager.yaml  input_topic={input_topic}, lidar={lidar_folder}, environment={environment}, test_results_dir={self._base_data_dir + "/" + bag_recorder_directory}')

    def _update_zones_config_yaml(self) -> None:
        """Params for zones_orchestrator_node (lidar_zones): builds profiles from
        TF + roi.yaml and serves /get_profiles. Needs only the lidar frame; the
        node resolves roi.yaml from its own package share when roi_config_path is
        left empty."""
        lidar_frame = self.lidar_params.get('frame')

        config = {
            'zones_orchestrator_node': {
                'ros__parameters': {
                    'lidar_frame': lidar_frame,
                }
            }
        }
        with self._zones_config_yaml.open('w') as f:
            yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False)
        print(f'  zones_orchestrator.yaml  lidar_frame={lidar_frame}')

    def _update_pointcloud_filter_yaml(self) -> None:
        """Params for pointcloud_filter_node (lidar_pointcloud_filter): hosts
        /roi_filter. Needs the lidar frame (for the viz rotation TF lookup) and the
        per-zone padding used by the projective filter. No cloud_topic — the node
        has no raw-cloud subscription (it filters clouds handed to it via the
        service)."""
        lidar_frame = self.lidar_params.get('frame')
        y_padding, z_padding = self._zone_paddings()

        config = {
            'pointcloud_filter_node': {
                'ros__parameters': {
                    'lidar_frame': lidar_frame,
                    'y_padding': y_padding,
                    'z_padding': z_padding,
                }
            }
        }
        with self._pointcloud_filter_yaml.open('w') as f:
            yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False)
        print(f'  pointcloud_filter.yaml   lidar_frame={lidar_frame}, '
              f'y_padding={y_padding}, z_padding={z_padding}')

    def _zone_paddings(self) -> tuple[dict[str, float], dict[str, float]]:
        """Build per-zone y/z padding dicts from the environment config.

        Zones missing y_padding/z_padding entries default to 0.0.
        """
        zones = self.environment_config.get('zones', [])
        zone_properties = self.environment_config.get('zone_properties', {})

        y_padding: dict[str, float] = {}
        z_padding: dict[str, float] = {}
        for zone_name in zones:
            props = zone_properties.get(zone_name, {})
            y_padding[zone_name] = float(props.get('y_padding', 0.0))
            z_padding[zone_name] = float(props.get('z_padding', 0.0))
        return y_padding, z_padding

    def _update_lidar_reporting_config_yaml(self) -> None:
        bag_directory = self.environment_config.get('bag_recorder_directory')
        lidar_folder = self.lidar_params.get('folder')
        lidar_frame = self.lidar_params.get('frame')

        lidar_cost = self.lidar_params.get('cost')
        lidar_horizontal_fov_deg = self.lidar_params.get('horizontal_fov_deg')
        lidar_vertical_fov_deg = self.lidar_params.get('vertical_fov_deg')
        if lidar_cost is not None:
            lidar_cost = float(lidar_cost)
        if lidar_horizontal_fov_deg is not None:
            lidar_horizontal_fov_deg = float(lidar_horizontal_fov_deg)
        if lidar_vertical_fov_deg is not None:
            lidar_vertical_fov_deg = float(lidar_vertical_fov_deg)


        environment = self.environment_config.get('name', '')
        with self._lidar_reporting_config_yaml.open() as f:
            config = yaml.safe_load(f)
        config['metrics_reporting_node']['ros__parameters']['lidar_frame'] = lidar_frame
        config['metrics_reporting_node']['ros__parameters']['lidar'] = lidar_folder
        config['metrics_reporting_node']['ros__parameters']['bag_directory'] = self._base_data_dir + "/" + bag_directory
        config['metrics_reporting_node']['ros__parameters']['metrics_results_dir'] = self._metrics_results_dir
        config['metrics_reporting_node']['ros__parameters']['lidar_cost'] = lidar_cost
        config['metrics_reporting_node']['ros__parameters']['lidar_horizontal_fov_deg'] = lidar_horizontal_fov_deg
        config['metrics_reporting_node']['ros__parameters']['lidar_vertical_fov_deg'] = lidar_vertical_fov_deg
        config['visualizer_node']['ros__parameters']['lidar'] = lidar_folder
        config['visualizer_node']['ros__parameters']['environment'] = environment
        with self._lidar_reporting_config_yaml.open('w') as f:
            yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False)
        print(f'  config.yaml            lidar_frame={lidar_frame}, lidar={lidar_folder}, environment={environment}')

    def _update_automation_manager_yaml(self) -> None:
        lidar_folder = self.lidar_params.get('folder')
        ros2_driver = self.lidar_params.get('ros2_driver')
        driver_command = self.lidar_params.get('driver_command')
        driver_config_file = self.lidar_params.get('driver_config_file')
        lidar_gui = self.lidar_params.get('lidar_gui')
        pointcloud_topic = self.lidar_params.get('point_cloud_topic')
        parameter_names = self.lidar_params.get('parameter_names', []) or []
        bag_recorder_directory = self.environment_config.get('bag_recorder_directory')
        bag_recording_duration = self.environment_config.get('bag_recording_duration')


        horizontal_fov_deg = self.lidar_params.get('horizontal_fov_deg')
        vertical_fov_deg = self.lidar_params.get('vertical_fov_deg')

        angles = self.utility_manager.compute_panning_angles(
            horizontal_fov_deg=horizontal_fov_deg,
            environment_config=self.environment_config,
            zone_engine=self._zone_engine,
        )

        with self._automation_manager_yaml.open() as f:
            config = yaml.safe_load(f)
        params = config['lidar_automation_manager']['ros__parameters']

        # Drop any sweep parameter sub-dicts from a previous lidar before adding the new ones.
        for stale_name in params.get('parameter_names', []) or []:
            params.pop(stale_name, None)

        params['lidar'] = lidar_folder
        params['ros2_driver'] = ros2_driver
        params['driver_command'] = driver_command
        params['driver_config_file'] = driver_config_file
        params['lidar_gui'] = lidar_gui
        params['bag_recorder_directory'] = self._base_data_dir + "/" + bag_recorder_directory
        params['bag_recording_duration'] = bag_recording_duration
        params['pointcloud_topic'] = pointcloud_topic
        params['angles'] = angles
        params['parameter_names'] = parameter_names
        for name in parameter_names:
            sub = self.lidar_params.get(name)
            if sub is not None:
                params[name] = sub

        with self._automation_manager_yaml.open('w') as f:
            yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False)
        print(f'  automation_manager.yaml lidar={lidar_folder}, angles={angles}, parameter_names={parameter_names}, bag_dir={bag_recorder_directory}, bag_duration={bag_recording_duration}')

    def _update_bringup_launch_yaml(self) -> None:
        # The bag destination no longer needs patching in here: the automation manager
        # records the bags itself and reads bag_recorder_directory from its own
        # config, written by _update_automation_manager_yaml.
        self.create_base_launch_file()

        # Carry over the last enable/disable-bag-recording choice. BAG_RECORDING comes from .env,
        # which just auto-loads into every recipe. This must run after create_base_launch_file,
        # which rewrites the file from scratch and would drop the comment markers this toggle
        # leaves behind.
        recording_enabled = os.environ.get('BAG_RECORDING', 'true').strip().lower() in (
            'true', '1', 'yes', 'on'
        )
        self.utility_manager.toggle_bag_recording(self._src_dir, recording_enabled)
        print(f'  lidar_test_bench_launch.yaml  bag recording '
              f'{"enabled" if recording_enabled else "disabled"} (BAG_RECORDING)')

    def _match_color(self, rgb: list[int]) -> str:
        best_name = 'grey'
        best_dist = float('inf')
        for entry in self._color_map:
            midpoint = [
                (entry['rgb_min'][i] + entry['rgb_max'][i]) / 2.0
                for i in range(3)
            ]
            dist = math.sqrt(sum((rgb[i] - midpoint[i]) ** 2 for i in range(3)))
            if dist < best_dist:
                best_dist = dist
                best_name = entry['name']
        return best_name

    def update_roi_regions(self) -> list:
        zones = self.environment_config.get('zones', [])
        zone_properties = self.environment_config.get('zone_properties', {})

        roi_zones = []
        for zone_name in zones:
            props = zone_properties.get(zone_name, {})
            rgb = props.get('color', [128, 128, 128])
            zone_type = props.get('type', 'planar')

            # Fields common to every geometry. Geometry-specific fields are
            # added below to match the ROILoader's per-type schema (planar →
            # PlanarZoneType, cylindrical → CylindricalZoneType).
            zone_entry = {
                'name': zone_name,
                'frame': props.get('frame', zone_name),
                'type': zone_type,
                'color': self._match_color(rgb),
            }

            zone_entry.update(self._zone_engine.roi_fields(zone_type, props))

            roi_zones.append(zone_entry)

        return roi_zones
