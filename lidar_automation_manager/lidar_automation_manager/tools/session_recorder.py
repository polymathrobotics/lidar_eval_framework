# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

import os
import shutil
import signal
import subprocess
import time
from pathlib import Path


class SessionRecorder:
    """Records a rosbag by driving `ros2 bag record` as a child process.

    Replaces the /session_recorder/{start,stop} service pair the manager used to
    call: those calls were fire-and-forget with a `time.sleep(duration)` between
    them and neither response was read, so an in-process subprocess is
    behaviourally identical and drops the dependency on that recorder.
    """

    # Ceiling on a single recording, so a bad config can't leave a recorder
    # running indefinitely.
    TIME_LIMIT_S = 1200

    def __init__(self, node):
        self._node = node
        self._proc = None

    def record(self, topics, bag_uri, duration_s):
        """Record `topics` for `duration_s` into `bag_uri`. Returns True on success."""
        duration_s = min(int(duration_s), self.TIME_LIMIT_S)
        bag_uri = Path(bag_uri)

        # `ros2 bag record -o` refuses an existing directory. Clearing rather than
        # sidestepping is deliberate: LidarBagPlayer enqueues every .mcap it finds
        # under a case folder, so a leftover bag would be replayed and would
        # overwrite the new bag's report.
        if bag_uri.exists():
            self._node.get_logger().warn(f'Replacing existing bag: {bag_uri}')
            shutil.rmtree(bag_uri)
        bag_uri.parent.mkdir(parents=True, exist_ok=True)

        self._node.get_logger().info(f'Recording {topics} for {duration_s}s -> {bag_uri}')
        # No -b/-d: a split bag lands as several .mcap files in one bag folder, and
        # LidarBagPlayer would queue the same bag once per file and replay it.
        self._proc = subprocess.Popen(
            ['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag_uri), *topics],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        time.sleep(duration_s)
        self.stop()

        if not (bag_uri / 'metadata.yaml').is_file():
            self._node.get_logger().error(f'No metadata.yaml in {bag_uri}; recording failed')
            return False
        return True

    def stop(self):
        """Stop the recorder, letting rosbag2 finalize the bag.

        SIGINT, not SIGTERM: rosbag2 writes metadata.yaml from its shutdown
        handler, and a bag without it is unreadable. The signal goes to the whole
        process group because `ros2 bag record` is a wrapper that spawns the
        actual recorder node as a child.
        """
        if self._proc is None or self._proc.poll() is not None:
            self._proc = None
            return

        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
        except ProcessLookupError:
            self._proc = None
            return

        try:
            self._proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            self._node.get_logger().warn('Recorder ignored SIGINT; killing (bag may be corrupt)')
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
            self._proc.wait(timeout=5)
        self._proc = None
