# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

import argparse
import math
from pathlib import Path

import yaml

from lidar_eval_fixture.harness_builder_interface import HarnessBuilderInterface
import polysetup_utils
from polysetup_utils import PolysetupUtils


_REPO_ROOT = Path(__file__).parent.parent
_LIDAR_CONFIGS_DIR = _REPO_ROOT / 'lidar_configs'
_ENVIRONMENT_CONFIGS_DIR = _REPO_ROOT / 'environment_configs'
_URDF_OUTPUT_PATH = _REPO_ROOT / 'lidar_transforms' / 'urdf' / 'lidar_bench.urdf'
_LIDAR_TEST_BENCH_YAML = _REPO_ROOT / 'lidar_test_bench' / 'config' / 'lidar_test_bench.yaml'
_LIDAR_FRAME_YAML = _REPO_ROOT / 'lidar_transforms' / 'config' / 'lidar_frame.yaml'
_LIDAR_REPORTING_CONFIG_YAML = _REPO_ROOT / 'lidar_reporting' / 'config' / 'config.yaml'
_ROI_YAML_PATH = _REPO_ROOT / 'lidar_transforms' / 'config' / 'roi.yaml'
_COLOR_MAP_PATH = Path(__file__).parent.parent / 'lidar_eval_fixture' / 'config' / 'color_map.yaml'
_LIDAR_AUTOMATION_MANAGER_YAML = _REPO_ROOT / 'lidar_automation_manager' / 'config' / 'automation_manager.yaml'
_LIDAR_TEST_BENCH_BRINGUP_YAML = _REPO_ROOT / 'lidar_test_bench_bringup' / 'launch' / 'lidar_test_bench_launch.yaml'



class ConfigureNewRun:

    def __init__(self, lidar_config: dict, environment_config: dict):
        self.lidar_config = lidar_config
        self.environment_config = environment_config
        self.lidar_params = lidar_config.get('lidar', {})
        self.utility_manager = PolysetupUtils()
        with _COLOR_MAP_PATH.open() as f:
            self._color_map = yaml.safe_load(f)['color_map']

        roi_zones = self.update_roi_regions()
        with _ROI_YAML_PATH.open('w') as f:
            yaml.safe_dump({'zones': roi_zones}, f, sort_keys=False, default_flow_style=False)



    def create_base_launch_file(self) -> None:

        launch = {
            'launch': [
                {
                    'node': {
                        'pkg': 'lidar_test_bench',
                        'exec': 'lidar_controller',
                        'name': 'lidar_controller',
                        'output': 'log',
                        'param': [{'from': '$(find-pkg-share lidar_test_bench)/config/lidar_test_bench.yaml'}],
                    }
                },
                {
                    'include': {
                        'file': '$(find-pkg-share lidar_test_bench_bringup)/launch/robot_description.launch.py',
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
                        'exec': 'grafana_reporter_node',
                        'name': 'grafana_reporter_node',
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
                        'pkg': 'lidar_transforms',
                        'exec': 'lidar_baseline_node',
                        'name': 'lidar_baseline_node',
                        'output': 'log',
                        'param': [{'from': '$(find-pkg-share lidar_transforms)/config/lidar_frame.yaml'}],
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
                    'include': {
                        'file': '$(find-pkg-share bag_recorder)/launch/bag_recorder_launch.yaml',
                        'arg': [{'name': 'bags_dest', 'value': 'DUMMY_BAGS_DEST'}],
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
        }
        with _LIDAR_TEST_BENCH_BRINGUP_YAML.open('w') as f:
            yaml.safe_dump(launch, f, sort_keys=False, default_flow_style=False)
        print(f'  lidar_test_bench_launch.yaml  recreated with base node set')


    def configure(self) -> None:
        print(f'Configuring run: lidar={self.lidar_params.get("frame")}, environment={self.environment_config.get("name", "unknown")}')
        self._build_urdf()
        self._update_lidar_test_bench_yaml()
        self._update_lidar_frame_yaml()
        self._update_lidar_reporting_config_yaml()
        self._update_automation_manager_yaml()
        self._update_bringup_launch_yaml()
        print('Done.')

    def _build_urdf(self) -> None:
        builder = HarnessBuilderInterface(
            environment_config_dict=self.environment_config,
            lidar_config_dict=self.lidar_config,
        )
        builder.build_harness(_URDF_OUTPUT_PATH)
        print(f'  lidar_bench.urdf       written to {_URDF_OUTPUT_PATH}')

    def _update_lidar_test_bench_yaml(self) -> None:


        # make the parameter overrides here if at all


        input_topic = self.lidar_params.get('point_cloud_topic')
        lidar_folder = self.lidar_params.get('folder')
        environment = self.environment_config.get('name', '')
        test_results_dir = self.environment_config.get('bag_recorder_directory')



        vertical_resolution = self.lidar_params.get('vertical_resolution_deg')
        horizontal_resolution = self.lidar_params.get('horizontal_resolution_deg')
        if vertical_resolution is not None:
            vertical_resolution = float(vertical_resolution)
        if horizontal_resolution is not None:
            horizontal_resolution = float(horizontal_resolution)

        with _LIDAR_TEST_BENCH_YAML.open() as f:
            config = yaml.safe_load(f)
        config['lidar_controller']['ros__parameters']['input_topic'] = input_topic
        config['lidar_controller']['ros__parameters']['lidar'] = lidar_folder
        config['lidar_controller']['ros__parameters']['environment'] = environment
        config['lidar_controller']['ros__parameters']['test_results_dir'] = test_results_dir
        config['lidar_controller']['ros__parameters']['horizontal_resolution_deg'] = horizontal_resolution
        config['lidar_controller']['ros__parameters']['vertical_resolution_deg'] = vertical_resolution



        with _LIDAR_TEST_BENCH_YAML.open('w') as f:
            yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False)
        print(f'  lidar_test_bench.yaml  input_topic={input_topic}, lidar={lidar_folder}, environment={environment}, test_results_dir={test_results_dir}')

    def _update_lidar_frame_yaml(self) -> None:
        input_topic = self.lidar_params.get('point_cloud_topic')
        lidar_frame = self.lidar_params.get('frame')

        y_padding, z_padding = self._zone_paddings()
        with _LIDAR_FRAME_YAML.open() as f:
            config = yaml.safe_load(f)
        config['lidar_bench_tf_broadcaster']['ros__parameters']['lidar_frame'] = lidar_frame
        config['lidar_baseline_node']['ros__parameters']['lidar_frame'] = lidar_frame
        config['lidar_baseline_node']['ros__parameters']['cloud_topic'] = input_topic
        config['lidar_baseline_node']['ros__parameters']['y_padding'] = y_padding
        config['lidar_baseline_node']['ros__parameters']['z_padding'] = z_padding

        with _LIDAR_FRAME_YAML.open('w') as f:
            yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False)
        print(f'  lidar_frame.yaml       lidar_frame={lidar_frame}, cloud_topic={input_topic}, '
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
        with _LIDAR_REPORTING_CONFIG_YAML.open() as f:
            config = yaml.safe_load(f)
        config['grafana_reporter_node']['ros__parameters']['lidar_frame'] = lidar_frame
        config['grafana_reporter_node']['ros__parameters']['lidar'] = lidar_folder
        config['grafana_reporter_node']['ros__parameters']['bag_directory'] = bag_directory
        config['grafana_reporter_node']['ros__parameters']['lidar_cost'] = lidar_cost
        config['grafana_reporter_node']['ros__parameters']['lidar_horizontal_fov_deg'] = lidar_horizontal_fov_deg
        config['grafana_reporter_node']['ros__parameters']['lidar_vertical_fov_deg'] = lidar_vertical_fov_deg
        config['visualizer_node']['ros__parameters']['lidar'] = lidar_folder
        config['visualizer_node']['ros__parameters']['environment'] = environment
        with _LIDAR_REPORTING_CONFIG_YAML.open('w') as f:
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
        )

        with _LIDAR_AUTOMATION_MANAGER_YAML.open() as f:
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
        params['bag_recorder_directory'] = bag_recorder_directory
        params['bag_recording_duration'] = bag_recording_duration
        params['pointcloud_topic'] = pointcloud_topic
        params['angles'] = angles
        params['parameter_names'] = parameter_names
        for name in parameter_names:
            sub = self.lidar_params.get(name)
            if sub is not None:
                params[name] = sub

        with _LIDAR_AUTOMATION_MANAGER_YAML.open('w') as f:
            yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False)
        print(f'  automation_manager.yaml lidar={lidar_folder}, angles={angles}, parameter_names={parameter_names}, bag_dir={bag_recorder_directory}, bag_duration={bag_recording_duration}')

    def _update_bringup_launch_yaml(self) -> None:
        self.create_base_launch_file()
        bags_dest = self.environment_config.get('bag_recorder_directory')
        with _LIDAR_TEST_BENCH_BRINGUP_YAML.open() as f:
            launch = yaml.safe_load(f)

        for entry in launch.get('launch', []):
            include = entry.get('include')
            if not include:
                continue
            if 'bag_recorder' not in include.get('file', ''):
                continue
            for arg in include.get('arg', []):
                if 'bags_dest' == arg.get('name'):
                    arg['value'] = bags_dest

        with _LIDAR_TEST_BENCH_BRINGUP_YAML.open('w') as f:
            yaml.safe_dump(launch, f, sort_keys=False, default_flow_style=False)
        print(f'  lidar_test_bench_launch.yaml  bags_dest={bags_dest}')

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

            zone_entry.update(polysetup_utils.build_zone_roi_fields(zone_type, props))

            roi_zones.append(zone_entry)

        return roi_zones




def main() -> None:
    parser = argparse.ArgumentParser(description='Configure a new lidar test bench run.')
    parser.add_argument('--lidar', required=True, help=f'Lidar config name. Available: {[p.stem for p in _LIDAR_CONFIGS_DIR.glob("*.yaml")]}')
    parser.add_argument('--environment', required=True, help=f'Environment config name. Available: {[p.stem for p in _ENVIRONMENT_CONFIGS_DIR.glob("*.yaml")]}')
    args = parser.parse_args()

    lidar_config_path = _LIDAR_CONFIGS_DIR / f'{args.lidar}.yaml'
    environment_config_path = _ENVIRONMENT_CONFIGS_DIR / f'{args.environment}.yaml'

    if not lidar_config_path.exists():
        available = [p.stem for p in _LIDAR_CONFIGS_DIR.glob('*.yaml')]
        raise FileNotFoundError(f'Lidar config "{args.lidar}" not found. Available: {available}')
    if not environment_config_path.exists():
        available = [p.stem for p in _ENVIRONMENT_CONFIGS_DIR.glob('*.yaml')]
        raise FileNotFoundError(f'Environment config "{args.environment}" not found. Available: {available}')

    with lidar_config_path.open() as f:
        lidar_config = yaml.safe_load(f)
    with environment_config_path.open() as f:
        environment_config = yaml.safe_load(f)

    ConfigureNewRun(lidar_config=lidar_config, environment_config=environment_config).configure()


if __name__ == '__main__':
    main()
