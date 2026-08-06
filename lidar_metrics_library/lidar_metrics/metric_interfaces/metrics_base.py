from abc import ABC, abstractmethod
import numpy as np
import yaml
import os
import json


class MetricsBase(ABC):
    """Single base for all lidar metrics (spatial and projective).

    The category (spatial vs projective) is declared in registry.yaml and used
    by the engine to dispatch the correct pre-bucketed cloud dict to each metric.
    Metrics receive a dict[zone_name, np.ndarray] of pre-filtered points per zone
    — no per-zone masking inside the metric is needed.

    Lifecycle (managed by LidarMetricsEngine):
      __init__  → setup() once, after config + profiles are wired in
      update(scan_cloud) → called once per scan; metric accumulates into its
                           own state (running sums, per-scan results, raw points,
                           etc.). Do not return a result here.
      compute() → called once at the end of the run, from engine.report().
                  Reduces the accumulated state to the final result dict.
      shutdown() → called once after compute(), before the next test run. Must
                   clear any accumulator state so the same plugin instance can
                   be safely reused for another run.
    """

    def __init__(
        self,
        pointcloud_by_zone: dict[str, np.ndarray],
        profiles=None,
        baseline_profiles=None,
    ):
        self.config: dict = {}
        self.pointcloud_by_zone = pointcloud_by_zone
        self.profiles = profiles
        self.baseline_profiles = baseline_profiles
        # Engine-injected sensor context. LidarMetricsEngine.create_plugins
        # overwrites these after instantiation and before setup() runs.
        # Defaults are here so direct instantiation in unit tests doesn't break.
        self.horizontal_resolution: float = 0.0
        self.vertical_resolution: float = 0.0

    def initialize_lib(self, config_path: str):
        self.load_config(config_path)

    def load_config(self, config_path: str) -> dict:
        if not isinstance(config_path, str) or not config_path:
            raise ValueError("config_path must be a non-empty string")

        if not os.path.isfile(config_path):
            raise FileNotFoundError(f"Config file not found: {config_path}")

        _, ext = os.path.splitext(config_path.lower())

        with open(config_path, "r", encoding="utf-8") as f:
            if ext == ".json":
                cfg = json.load(f)
            elif ext in (".yaml", ".yml"):
                if yaml is None:
                    raise ImportError("pyyaml is required to load YAML configs (pip install pyyaml)")
                cfg = yaml.safe_load(f)
            else:
                raise ValueError(f"Unsupported config extension: {ext} (use .json/.yaml/.yml)")

        if cfg is None:
            cfg = {}
        if not isinstance(cfg, dict):
            raise ValueError("Config file must parse to a dictionary at the top level")

        self.config = cfg

    @abstractmethod
    def setup(self) -> None:
        """One-time initialization. Read thresholds from self.config and any
        profile-derived constants (zone grids, expected geometry, etc.)."""

    @abstractmethod
    def update(self, pointcloud_by_zone: dict[str, np.ndarray]) -> None:
        """Called once per scan. Accumulate this scan's contribution into the
        metric's internal state. Must not return a result."""

    @abstractmethod
    def compute(self):
        """Called once at end of run. Reduce accumulated state to a final
        result dict and return it."""

    @abstractmethod
    def shutdown(self) -> None:
        """Called once after compute(). Clear accumulator state so the plugin
        instance can be reused for a subsequent test run."""
