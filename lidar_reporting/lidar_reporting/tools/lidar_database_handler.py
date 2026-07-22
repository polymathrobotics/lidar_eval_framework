# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

import importlib
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
import yaml

class LidarDatabaseHandler:

    def __init__(self, lidar_metadata: dict[str, Any] | None = None):
        self._database_handler = None
        self.authenticated = False
        self._lidar_metadata = lidar_metadata or {}

        share_dir = get_package_share_directory('lidar_reporting')
        config_path = Path(share_dir) / 'config' / 'database_registry.yaml'

        self.configure_backend(config_path)

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

    def configure_backend(self, config_path: Path) -> None:
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

        self._database_handler.sync(test_data, rosbags, self._lidar_metadata)

    def push_visualization(self, env: str, lidar: str, case: str, blocks: list) -> None:
        self._ensure_authenticated()
        if not self.available:
            return

        self._database_handler.push_visualization_to_case(env, lidar, case, blocks)

    def _ensure_authenticated(self) -> None:
        if self.authenticated:
            return
        try:
            self.authenticate()
        except Exception:
            self.authenticated = False
