# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""
High-level tests for LidarBagPlayer — how a bag tree becomes an ordered play queue.

Playback itself shells out to `ros2 bag play`, so these cover the queue building
and the report-path derivation, not the subprocess.
"""

from pathlib import Path

from lidar_eval_orchestrator.tools.bag_runner import LidarBagPlayer
import pytest


def touch_bag(root, relative_path):
    """Create a rosbag2 directory holding one .mcap at `relative_path`."""
    bag_dir = root / relative_path
    bag_dir.mkdir(parents=True, exist_ok=True)
    (bag_dir / 'data_0.mcap').write_bytes(b'')
    return bag_dir


@pytest.fixture
def bag_root(tmp_path):
    """One lidar's bag tree with a base, a parameter sweep and two angles."""
    root = tmp_path / 'bags' / 'at128'
    touch_bag(root, 'base/rosbag2_base')
    touch_bag(root, 'parameter_configs/return_mode/dual/rosbag2_dual')
    touch_bag(root, 'angles/angle=30/rosbag2_a30')
    touch_bag(root, 'angles/angle=15/rosbag2_a15')
    return root


def test_refresh_orders_the_queue_base_then_configs_then_angles(bag_root):
    player = LidarBagPlayer(str(bag_root))

    player.refresh_file_list()

    assert player.has_next()
    cases = [Path(q).relative_to(bag_root).parent.as_posix() for q in player._play_queue]
    assert cases == [
        'base',
        'parameter_configs/return_mode/dual',
        'angles/angle=15',
        'angles/angle=30',
    ]


def test_refresh_on_a_missing_directory_leaves_an_empty_queue(tmp_path):
    player = LidarBagPlayer(str(tmp_path / 'nope'))

    player.refresh_file_list()

    assert not player.has_next()
    assert player.next_bag_path() is None
    assert player.next_bag_report_info() is None


def test_report_info_maps_each_case_to_its_output_folder_and_stem(bag_root):
    player = LidarBagPlayer(str(bag_root))
    player.refresh_file_list()

    infos = []
    for index in range(len(player._play_queue)):
        player._idx = index
        infos.append(player.next_bag_report_info())

    assert infos == [
        ('at128/base', 'report'),
        ('at128/parameter_configs/return_mode', 'dual_report'),
        ('at128/angles', 'angle=15_report'),
        ('at128/angles', 'angle=30_report'),
    ]


def test_next_bag_angle_is_parsed_from_the_angle_folder(bag_root):
    player = LidarBagPlayer(str(bag_root))
    player.refresh_file_list()

    player._idx = 0
    assert player.next_bag_angle() is None      # base carries no angle
    player._idx = 2
    assert player.next_bag_angle() == 15


def test_next_bag_name_is_the_lidar_relative_path_flattened(bag_root):
    player = LidarBagPlayer(str(bag_root))
    player.refresh_file_list()

    assert player.next_bag_name() == 'at128_base_rosbag2_base'


def test_a_fresh_player_reports_no_playback_in_progress(bag_root):
    player = LidarBagPlayer(str(bag_root))

    assert not player.is_playing()
    assert not player.in_startup_grace()
    assert player.last_exit_code() is None
    player.stop()   # no-op rather than an error when nothing is running
