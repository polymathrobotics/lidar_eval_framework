# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

import os
import signal
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace

import yaml

from lidar_automation_manager.tools.session_recorder import SessionRecorder


class DriverManager:

    def __init__(self, node):
        self._node = node
        self._proc = None
        self._recorder = SessionRecorder(node)

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


    def _case_dir(self, case=None):
        """Where a case's bag lives.

        This is the tree LidarBagPlayer walks to recover case identity — it parses
        `base` / `angles/angle=<n>` / `parameter_configs/<param>/<value>`
        positionally, so the layout and those literal folder names are load-bearing.
        """
        lidar_root = Path(self._node.bag_recorder_directory) / self._node.lidar

        if case is None:
            return lidar_root / 'base'
        if 'angle' == case.test_type:
            return lidar_root / 'angles' / f'angle={int(case.angle)}'
        return lidar_root / 'parameter_configs' / case.parameter / str(case.value)


    def _bag_name(self, case=None):
        """Bag folder name. Redundant with the case folder above it, but it becomes
        the zip filename when the bag is uploaded, so keep it readable."""
        if case is None:
            return 'base'
        if 'angle' == case.test_type:
            sign = 'neg' if case.angle < 0 else ''
            return f'angle_{sign}{abs(int(case.angle))}'
        return f'{case.parameter}_{str(case.value).replace(".", "_")}'


    def record_bag(self, case=None):

        self.launch_driver()

        if not self.wait_for_topic(self._node.pointcloud_topic):
            self._node.get_logger().error(
                f'Pointcloud topic {self._node.pointcloud_topic} did not appear; skipping recording')
            return

        # Recorded straight into its final case folder — no post-hoc "find the folder
        # we just wrote and move it" step to get wrong.
        self._recorder.record(
            topics=[self._node.pointcloud_topic],
            bag_uri=self._case_dir(case) / self._bag_name(case),
            duration_s=self._node.bag_recording_duration,
        )


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
            gui_case = SimpleNamespace(test_type='parameter_gui', parameter=name, value=value)
            self.record_bag(gui_case)
