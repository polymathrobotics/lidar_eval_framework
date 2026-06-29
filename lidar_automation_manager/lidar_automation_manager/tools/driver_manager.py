# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

import os
import re
import shutil
import signal
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace

import yaml
from polymath_core_msgs.srv import StartBagRecording, StopBagRecording


# bag_recorder names every recording folder like: corp-<host>__<YYYY-MM-DDTHH-MM-SSZ>__<suffix>
# Filtering on this prefix is the only safe way to pick "the bag just recorded" — using
# `most recently modified directory in COOP_HANGOUT_BAG/` alone can pick up other bag
# folders that had their mtime bumped by unrelated cp/restore/edit operations.
_BAG_NAME_RE = re.compile(r'^corp-.+__\d{4}-\d{2}-\d{2}T\d{2}-\d{2}-\d{2}Z__.+$')


class DriverManager:

    def __init__(self, node):
        self._node = node
        self._proc = None

    def launch_driver(self):
        if self._proc is not None and self._proc.poll() is None:
            self._node.get_logger().warn(f'{self._node.driver_command} is already running')
            return

        self._node.get_logger().info(f'Launching driver: {self._node.driver_command}')
        self._proc = subprocess.Popen(
            self._node.driver_command.split(),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )


    def set_driver_config_to_default(self):
        if not self._node.driver_config_file or not self._node.defaults:
            return

        with open(self._node.driver_config_file, 'r') as f:
            config = yaml.safe_load(f) or {}

        for path, value in self._node.defaults.values():
            self._set_at_path(config, path, value)

        with open(self._node.driver_config_file, 'w') as f:
            yaml.safe_dump(config, f, default_flow_style=False)


    def kill_driver_processes(self):
        # pkill -f only matches `ros2 (run|launch) <driver>` in the wrapper's cmdline; the node
        # binaries it spawns (e.g. ob_camera_node) don't contain that string and would survive,
        # leaving the sensor / multicast port bound and a stale publisher in the ROS graph that
        # makes subsequent runs record empty bags. Kill the whole process group instead — we
        # launched with start_new_session=True so the launcher and all its descendants share it.
        if self._proc is not None:
            try:
                pgid = os.getpgid(self._proc.pid)
                os.killpg(pgid, signal.SIGTERM)
                time.sleep(2)
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            except ProcessLookupError:
                pass
            self._proc = None

        # Fallback: kill any orphan wrappers from prior runs that aren't in our process group.
        pattern = f'ros2 (run|launch) {self._node.ros2_driver}'
        self._node.get_logger().info(f'Killing driver processes matching: {pattern}')
        subprocess.run(['pkill', '-TERM', '-f', pattern], check=False)
        time.sleep(1)
        subprocess.run(['pkill', '-KILL', '-f', pattern], check=False)
        self._node.get_logger().info('Driver processes stopped')



    def set_driver_param(self, case):
        with open(self._node.driver_config_file, 'r') as f:
            config = yaml.safe_load(f)

        self._set_at_path(config, case.path, case.value)

        with open(self._node.driver_config_file, 'w') as f:
            yaml.safe_dump(config, f, default_flow_style=False)


    def _set_at_path(self, config, path, value):
        keys = path.split('.')
        target = config
        for k in keys[:-1]:
            target = target[int(k)] if k.isdigit() else target[k]
        last = keys[-1]
        target[int(last) if last.isdigit() else last] = value


    def wait_for_topic(self, topic, timeout=30.0):
        # count_publishers() reflects live publishers, unlike get_topic_names_and_types() which
        # can return stale entries for publishers that have died but not yet been pruned from
        # the graph — that lie would let us start the bag against an empty topic.
        start = time.time()
        while time.time() - start < timeout:
            if 0 < self._node.count_publishers(topic):
                return True
            time.sleep(0.5)
        return False


    def record_bag(self, bag_suffix=''):

        self.launch_driver()

        if not self.wait_for_topic(self._node.pointcloud_topic):
            self._node.get_logger().error(
                f'Pointcloud topic {self._node.pointcloud_topic} did not appear; skipping recording')
            return

        request = StartBagRecording.Request()
        request.bag_suffix = bag_suffix
        request.include = [self._node.pointcloud_topic]
        request.record_duration = self._node.bag_recording_duration
        self._node.start_bag_client.call_async(request)
        time.sleep(self._node.bag_recording_duration)
        self._node.stop_bag_client.call_async(StopBagRecording.Request())
        time.sleep(2)


    def move_to_directory(self, case=None):
        bags_dest = Path(self._node.bag_recorder_directory)
        lidar_root = bags_dest / self._node.lidar

        # Only consider folders whose names match the bag_recorder's output pattern.
        # Then pick the most recently modified one — that's the bag just recorded.
        candidates = [
            d for d in bags_dest.iterdir()
            if d.is_dir() and _BAG_NAME_RE.match(d.name)
        ]
        if not candidates:
            return
        latest = max(candidates, key=lambda p: p.stat().st_mtime)

        if case is None:
            dest = lidar_root / 'base' / latest.name
        elif 'angle' == case.test_type:
            dest = lidar_root / 'angles' / f'angle={int(case.angle)}' / latest.name
        else:
            dest = lidar_root / 'parameter_configs' / case.parameter / str(case.value) / latest.name

        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(latest), str(dest))


    def prompt_and_record_gui_params(self, gui_path):
        if not gui_path:
            return {}

        self._node.get_logger().info(f'Launching GUI: {gui_path}')
        gui_cwd = str(Path(gui_path).parent) or '.'
        subprocess.Popen([gui_path], start_new_session=True, cwd=gui_cwd)

        while True:
            name = input('Change a parameter in the GUI, then enter its name (empty to finish): ').strip()
            if not name:
                break
            value = input(f'Enter the value you set for {name}: ').strip()

            self.set_driver_config_to_default()
            self.record_bag(bag_suffix=f'{name}_{value}')
            gui_case = SimpleNamespace(test_type='parameter_gui', parameter=name, value=value)
            self.move_to_directory(case=gui_case)
