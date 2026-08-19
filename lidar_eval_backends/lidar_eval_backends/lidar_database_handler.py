import importlib
from importlib import resources
from pathlib import Path
from typing import Any

import yaml

class LidarDatabaseHandler:

    def __init__(self, lidar_metadata: dict[str, Any] | None = None,
                 registry_path: Path | None = None,
                 credentials: dict | None = None):
        self._database_handler = None
        self.authenticated = False
        self._lidar_metadata = lidar_metadata or {}

        if registry_path is None:
            registry_path = resources.files('lidar_eval_backends') / 'database_registry.yaml'
        self.configure_backend(registry_path)


        if credentials is not None:
            self.load_credentials(credentials)

    @property
    def available(self) -> bool:
        return self._database_handler is not None and self._database_handler.available

    def authenticate(self) -> None:

        if self._database_handler is None:
            self.authenticated = False
            raise RuntimeError("No database backend configured.")
        try:
            self._database_handler.authenticate()
            self.authenticated = True
        except Exception:
            self.authenticated = False
            raise

    def load_credentials(self, credentials: dict) -> None:
        # Inject an opaque credential blob straight into the backend (read path — e.g. PolyView on
        # Streamlit Cloud passing st.secrets), bypassing the interactive authenticate() flow.
        if self._database_handler is not None:
            self._database_handler.load_credentials(credentials)

    def configure_backend(self, config_path) -> None:
        # .open() works for both a pathlib.Path and an importlib.resources traversable.
        with config_path.open('r') as file:
            config_data = yaml.safe_load(file)

        registry = config_data.get("database_registry", [])

        for backend in registry:
            if backend.get("enabled") is True:
                module_name = backend.get("executable")
                class_name = backend.get("class")

                module_path = f"lidar_eval_backends.database_backends.{module_name}"
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

    # ---- Reading (PolyView) — delegate to the backend; no interactive auth ----

    def clear_cache(self) -> None:
        if self._database_handler is not None:
            self._database_handler.clear_cache()

    def retrieve_environments(self) -> list[str]:
        return self._database_handler.retrieve_environments() if self._database_handler else []

    def retrieve_env_data(self, env_name: str) -> dict:
        return self._database_handler.retrieve_env_data(env_name) if self._database_handler else {}

    def retrieve_visualization_data(self, env_name: str, lidar_name: str, case_path: str) -> dict:
        if self._database_handler is None:
            return {}
        return self._database_handler.retrieve_visualization_data(env_name, lidar_name, case_path)

    def retrieve_bag_download_link(self, env_name: str, lidar_name: str, case_path: str) -> str | None:
        if self._database_handler is None:
            return None
        return self._database_handler.retrieve_bag_download_link(env_name, lidar_name, case_path)

    def _ensure_authenticated(self) -> None:
        if self.authenticated:
            return
        try:
            self.authenticate()
        except Exception:
            self.authenticated = False
