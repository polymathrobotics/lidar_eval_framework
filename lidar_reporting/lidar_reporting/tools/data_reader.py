# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

from pathlib import Path
from typing import Any

import yaml


class DataReader:
    def __init__(self, metrics_results_dir: Path, rosbag_dir: Path | str | None = None):
        self._results_dir = Path(metrics_results_dir)
        # Optional: consumers that only read metrics (e.g. visualizer_node) need no bags.
        self._rosbag_dir = Path(rosbag_dir) if rosbag_dir else None

    def load(self) -> dict[str, dict[str, dict[str, Any]]]:
        result: dict[str, dict[str, dict[str, Any]]] = {}
        if not self._results_dir or not self._results_dir.is_dir():
            return result
        for env_folder in sorted(self._results_dir.iterdir()):
            if not env_folder.is_dir():
                continue
            env_data = self._load_environment(env_folder)
            if env_data:
                result[env_folder.name] = env_data
        return result

    def _load_environment(self, env_folder: Path) -> dict[str, dict[str, Any]]:
        result: dict[str, dict[str, Any]] = {}
        for lidar_folder in sorted(env_folder.iterdir()):
            if not lidar_folder.is_dir():
                continue
            lidar_data = self._load_lidar(lidar_folder)
            if lidar_data:
                result[lidar_folder.name] = lidar_data
        return result


    def load_rosbags(self) -> dict[str, dict[str, list[Path]]]:
        """Map every recorded rosbag under the bag root to its lidar + case.

        ``self._rosbag_dir`` is one environment's bag root (the env config's
        ``bag_recorder_directory``); its layout mirrors the metrics tree minus
        the env level::

            <lidar>/<case.../>/rosbag2_<timestamp>/{metadata.yaml, *.mcap}

        Returns ``{lidar_name: {case_name: [bag_dir, ...]}}`` where each
        ``bag_dir`` is the ``rosbag2_<timestamp>`` directory (the folder to
        upload) and ``case_name`` matches the metrics reader's convention
        (e.g. ``"base"``, ``"angles/angle=15"``), so the two trees line up.
        """
        result: dict[str, dict[str, list[Path]]] = {}
        if not self._rosbag_dir or not self._rosbag_dir.is_dir():
            return result
        for lidar_folder in sorted(self._rosbag_dir.iterdir()):
            if not lidar_folder.is_dir():
                continue
            # A rosbag2 directory is the one holding metadata.yaml; rglob finds
            # them at any case-nesting depth (base, angles/<a>, parameter_configs/...).
            for metadata in sorted(lidar_folder.rglob('metadata.yaml')):
                bag_dir = metadata.parent
                case = self._bag_case_name(bag_dir, lidar_folder)
                result.setdefault(lidar_folder.name, {}).setdefault(case, []).append(bag_dir)
        return result

    @staticmethod
    def _bag_case_name(bag_dir: Path, lidar_folder: Path) -> str:
        """Case name for a bag: the path from the lidar folder down to the
        bag's parent (the case folder), slash-joined — matching _case_name.
        Falls back to the bag's own name if it sits directly under the lidar
        folder (no case folder)."""
        case_folder = bag_dir.parent
        if case_folder == lidar_folder:
            return bag_dir.name
        return '/'.join(case_folder.relative_to(lidar_folder).parts)

    def _load_lidar(self, lidar_folder: Path) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for yaml_file in sorted(lidar_folder.rglob('*.yaml')):
            try:
                with open(yaml_file) as f:
                    data = yaml.safe_load(f)
            except (yaml.YAMLError, OSError) as e:
                # A report.yaml caught mid-write by lidar_controller scans as malformed.
                # Skip it this pass (the next load sees the completed file) rather than
                # letting one half-written file take down the calling node.
                print(f'[DataReader] skipping unreadable {yaml_file}: {e}')
                continue
            if not data:
                continue
            result[self._case_name(yaml_file, lidar_folder)] = data
        return result

    def most_recent_case(self) -> tuple[str, str, str] | None:
        newest_mtime = 0.0
        newest: tuple[str, str, str] | None = None
        if not self._results_dir.is_dir():
            return None
        for yaml_file in self._results_dir.rglob('*.yaml'):
            mtime = yaml_file.stat().st_mtime
            if mtime <= newest_mtime:
                continue
            parts = yaml_file.relative_to(self._results_dir).parts
            if len(parts) < 3:
                continue
            env, lidar = parts[0], parts[1]
            case = self._case_name(yaml_file, self._results_dir / env / lidar)
            newest_mtime = mtime
            newest = (env, lidar, case)
        return newest

    @staticmethod
    def _case_name(yaml_file: Path, lidar_folder: Path) -> str:
        parts = list(yaml_file.relative_to(lidar_folder).parts)
        stem = parts[-1]
        if stem.endswith('_report.yaml'):
            stem = stem[: -len('_report.yaml')]
        elif stem.endswith('.yaml'):
            stem = stem[: -len('.yaml')]
        if stem == 'report':
            parts = parts[:-1]
        else:
            parts[-1] = stem
        return '/'.join(parts)
