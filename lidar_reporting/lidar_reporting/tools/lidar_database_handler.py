# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

import importlib
from pathlib import Path
from typing import Any
import yaml


from lidar_reporting.tools.database_backends.google_services_handler import GoogleServicesHandler


class LidarDatabaseHandler:

    def __init__(self, lidar_metadata: dict[str, Any] | None = None):
        self._database_handler = None
        self.authenticated = False
        self._lidar_metadata = lidar_metadata or {}
        self.configure_backend("database_registry.yaml")

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

    def configure_backend(self, filename: str) -> None:
        config_path = Path(__file__).parent / filename

        with open(config_path, 'r') as file:
            config_data = yaml.safe_load(file)

        registry = config_data.get("database_registry", [])

        for backend in registry:
            if backend.get("enabled") is True:
                module_name = backend.get("executable")
                class_name = backend.get("class")

                module_path = f"lidar_reporting.tools.database_backends.{module_name}"
                module = importlib.import_module(module_path)

                backend_class = getattr(module, class_name)
                self._database_handler = backend_class()
                break


    def sync(
        self,
        test_data: dict[str, dict[str, dict[str, Any]]],
        rosbags: dict[str, dict[str, list]] | None = None,
    ) -> None:
        self._ensure_authenticated()
        if not self.available:
            return


        self._database_handler.sync(test_data, rosbags)

    # make this a wrapper as well

    def push_visualization(self, env: str, lidar: str, case: str, blocks: list) -> None:
        self._ensure_authenticated()
        if not self.available:
            return

        # include below logic in the push_visualization_to_case method her

        self._database_handler.push_visualization_to_case(env, lidar, case, blocks)

    def _ensure_authenticated(self) -> None:
        if self.authenticated:
            return
        try:
            self.authenticate()
        except Exception:
            self.authenticated = False

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
