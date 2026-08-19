# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""High-level tests for DataReader — walking the metrics tree and the bag tree."""

import os

import pytest
import yaml

from lidar_reporting.tools.data_reader import DataReader


def write_report(root, relative_path, data=None):
    """Write a report.yaml at `relative_path` under `root`, creating parents."""
    path = root / relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(data if data is not None else {'wall': {'M': {'sub': 1.0}}}))
    return path


def touch_bag(root, relative_path):
    """Create a rosbag2 directory (metadata.yaml + an mcap) at `relative_path`."""
    bag_dir = root / relative_path
    bag_dir.mkdir(parents=True, exist_ok=True)
    (bag_dir / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}\n')
    (bag_dir / 'data_0.mcap').write_bytes(b'')
    return bag_dir


@pytest.fixture
def results_dir(tmp_path):
    """A metrics tree covering all three case layouts the bench produces."""
    root = tmp_path / 'metrics_results'
    write_report(root, 'E1/at128/base/report.yaml')
    write_report(root, 'E1/at128/angles/angle=15_report.yaml')
    write_report(root, 'E1/at128/parameter_configs/return_mode/dual_report.yaml')
    write_report(root, 'E1/jt128/base/report.yaml')
    write_report(root, 'E2/at128/base/report.yaml')
    return root


def test_load_returns_an_env_lidar_case_tree(results_dir):
    tree = DataReader(results_dir).load()

    assert sorted(tree) == ['E1', 'E2']
    assert sorted(tree['E1']) == ['at128', 'jt128']
    assert sorted(tree['E1']['at128']) == [
        'angles/angle=15',
        'base',
        'parameter_configs/return_mode/dual',
    ]
    # The leaf is the report's own contents, untouched.
    assert tree['E1']['at128']['base'] == {'wall': {'M': {'sub': 1.0}}}


def test_load_on_a_missing_directory_is_empty(tmp_path):
    assert DataReader(tmp_path / 'nope').load() == {}


def test_load_skips_a_half_written_report(results_dir, capsys):
    write_report(results_dir, 'E1/at128/base/report.yaml')
    (results_dir / 'E1' / 'jt128' / 'base' / 'report.yaml').write_text('wall: {unclosed: [1, 2')

    tree = DataReader(results_dir).load()

    # The malformed file drops out; the rest of the tree still loads.
    assert 'jt128' not in tree['E1']
    assert 'base' in tree['E1']['at128']
    assert 'skipping unreadable' in capsys.readouterr().out


def test_most_recent_case_picks_the_newest_report(results_dir):
    newest = results_dir / 'E1' / 'at128' / 'angles' / 'angle=15_report.yaml'
    for path in results_dir.rglob('*.yaml'):
        os.utime(path, (1_700_000_000, 1_700_000_000))
    os.utime(newest, (1_800_000_000, 1_800_000_000))

    assert DataReader(results_dir).most_recent_case() == ('E1', 'at128', 'angles/angle=15')


def test_load_rosbags_mirrors_the_metrics_case_names(tmp_path, results_dir):
    bags = tmp_path / 'bags'
    base_bag = touch_bag(bags, 'at128/base/rosbag2_1')
    angle_bag = touch_bag(bags, 'at128/angles/angle=15/rosbag2_2')
    loose_bag = touch_bag(bags, 'jt128/rosbag2_3')

    found = DataReader(results_dir, bags).load_rosbags()

    assert found['at128']['base'] == [base_bag]
    assert found['at128']['angles/angle=15'] == [angle_bag]
    # A bag sitting directly under the lidar folder falls back to its own name.
    assert found['jt128']['rosbag2_3'] == [loose_bag]


def test_load_rosbags_without_a_bag_dir_is_empty(results_dir):
    assert DataReader(results_dir).load_rosbags() == {}
