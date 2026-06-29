# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

from typing import Any

from lidar_reporting.tools.database_backends.google_services_handler import GoogleServicesHandler


class LidarDatabaseHandler:

    def __init__(self, lidar_metadata: dict[str, Any] | None = None):
        self._database_handler = GoogleServicesHandler()
        self.authenticated = False
        self._lidar_metadata = lidar_metadata or {}

    @property
    def available(self) -> bool:
        return self.authenticated

    def authenticate(self) -> None:
        try:
            self._database_handler.authenticate()
            self.authenticated = True
        except Exception:
            self.authenticated = False
            raise


    # make this a wrapper to a lower level sync function

    def sync(
        self,
        test_data: dict[str, dict[str, dict[str, Any]]],
        rosbags: dict[str, dict[str, list]] | None = None,
    ) -> None:
        self._ensure_authenticated()
        if not self.available:
            return

        rosbags = rosbags or {}
        for env_name, lidar_data in test_data.items():
            self._database_handler.create_environment_folder(env_name)
            summary_cases: list[tuple[str, str, dict[str, Any]]] = []
            for lidar_name, case_data in lidar_data.items():
                self._database_handler.create_lidar_folder(env_name, lidar_name)
                for case_path, metrics in case_data.items():
                    flat_metrics = self._flatten_metrics(metrics)
                    if not flat_metrics:
                        continue
                    merged = {**self._lidar_metadata, **flat_metrics}
                    # Bags are keyed by (lidar, case) only — the bag root is a single
                    # env, so there's no env level to match. Missing → no bag uploaded.
                    case_bags = rosbags.get(lidar_name, {}).get(case_path, [])
                    self._database_handler.create_folder_per_case(
                        env_name, lidar_name, case_path, merged, case_bags
                    )
                    summary_cases.append((lidar_name, case_path, merged))
            # One summary sheet per env (one row per case) so the dashboard loads the whole
            # environment in a single read instead of one read per case.
            self._database_handler.write_environment_summary(env_name, summary_cases)


    # make this a wrapper as well

    def push_visualization(self, env: str, lidar: str, case: str, blocks: list) -> None:
        self._ensure_authenticated()
        if not self.available:
            return

        # include below logic in the push_visualization_to_case method here

        rows = self._blocks_to_rows(blocks)
        if not rows:
            return

        ###

        self._database_handler.push_visualization_to_case(env, lidar, case, rows)

    def _ensure_authenticated(self) -> None:
        if self.authenticated:
            return
        try:
            self.authenticate()
        except Exception:
            self.authenticated = False

    @staticmethod
    def _blocks_to_rows(blocks: list) -> list[list[str]]:
        rows: list[list[str]] = []
        for block in blocks:
            btype = block.get('type')
            if not btype:
                continue
            rich_text = block.get(btype, {}).get('rich_text', [])
            if not rich_text:
                continue
            text = rich_text[0].get('text', {}).get('content', '')
            if not text:
                continue
            rows.append([text])
        return rows

    @classmethod
    def _flatten_metrics(cls, metrics: dict[str, Any]) -> dict[str, Any]:
        """Flatten the nested report ({zone}/{metric}/{sub}: value) into
        slash-joined column keys. Descends arbitrarily deep so it tracks the
        report structure; skips visualization blocks and per-cell dead-cell keys
        wherever they appear."""
        flat: dict[str, Any] = {}
        cls._flatten_into(metrics, '', flat)
        return flat

    @classmethod
    def _flatten_into(cls, node: Any, prefix: str, flat: dict[str, Any]) -> None:
        if not isinstance(node, dict):
            return
        for key, val in node.items():
            if key == 'visualization' or 'dead_cell_' in str(key) or 'worst_point_' in str(key):
                continue
            path = f'{prefix}/{key}' if prefix else str(key)
            if isinstance(val, dict):
                cls._flatten_into(val, path, flat)
            elif isinstance(val, (int, float, str)):
                flat[path] = val
