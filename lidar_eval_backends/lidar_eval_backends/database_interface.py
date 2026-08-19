from abc import ABC, abstractmethod
from typing import Any


class DatabaseInterface(ABC):
    """Full read + write contract for a database backend.

    One backend implements all of it; consumers use the half they need — the reporter
    writes, PolyView reads. Read return shapes are part of the contract.
    """


    # Abstract Methods for authentication

    @abstractmethod
    def authenticate(self) -> None:
        """Populate this backend's credentials for the write path — e.g. fetch a service-account
        key from a secrets store (interactive backends may prompt). PolyView's read path skips
        this and injects credentials directly via load_credentials()."""
        raise NotImplementedError

    @abstractmethod
    def load_credentials(self, credentials: dict) -> None:
        """Receive an opaque credential/config blob and pull out what this backend needs
        (Google: the Drive root folder id + the service-account fields). PolyView injects the
        blob directly; the write path can use this or authenticate()."""
        raise NotImplementedError


    # Abstract Methods for writing data to Database

    @abstractmethod
    def sync(
        self,
        test_data: dict[str, dict[str, dict[str, Any]]],
        rosbags: dict[str, dict[str, list]] | None = None,
        lidar_metadata: dict[str, Any] | None = None,
    ) -> None:
        """Push the metrics tree (env -> lidar -> case -> data) to the backend."""
        raise NotImplementedError

    @abstractmethod
    def push_visualization_to_case(
        self,
        env_name: str,
        lidar_name: str,
        case_name: str,
        blocks: list[list[str]],
    ) -> None:
        """Attach visualization blocks to an already-synced case."""
        raise NotImplementedError


    # Abstract Methods for reading data from Database

    @property
    @abstractmethod
    def available(self) -> bool:
        """True when the backend is configured/usable (e.g. creds present)."""
        raise NotImplementedError

    @abstractmethod
    def clear_cache(self) -> None:
        """Drop cached listings so the next read re-fetches. No-op if cache-less."""
        raise NotImplementedError

    @abstractmethod
    def retrieve_environments(self) -> list[str]:
        """Environment names available in the store."""
        raise NotImplementedError

    @abstractmethod
    def retrieve_env_data(self, env_name: str) -> dict[str, dict]:
        """One env's full metrics tree: {lidar: {case...: {zone: {Metric: {sub: number}}}}}
        plus a per-case '__global__' bucket. {} if missing/unavailable."""
        raise NotImplementedError

    @abstractmethod
    def retrieve_visualization_data(self, env_name: str, lidar_name: str, case_path: str) -> dict:
        """Per-case 3D payload: {roi_cloud, filtered_roi_cloud, fitted_planes, dead_cells,
        worst_points, orientation, profile_plane}."""
        raise NotImplementedError

    @abstractmethod
    def retrieve_bag_download_link(self, env_name: str, lidar_name: str, case_path: str) -> str | None:
        """Direct download URL for the case's rosbag, or None if none/unsupported."""
        raise NotImplementedError
