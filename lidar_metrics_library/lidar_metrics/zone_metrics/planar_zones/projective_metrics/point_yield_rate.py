from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class PointYieldRate(MetricsBase):

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self.valid_points = 0
        self.expected_num_points = 0
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        return

    def update(self, pointcloud_by_zone) -> None:
        self.pointcloud_by_zone = pointcloud_by_zone

    def compute(self) -> dict[str, float]:
        # Aggregate yield across the union of all zones. Numerator and denominator
        # are currently both derived from the filtered cloud, so the rate is bounded
        # near 1.0 — see the bug note in registry/docs about needing a beam-geometry
        # derived expected count for a meaningful rate.
        self.valid_points = 0
        self.expected_num_points = 0
        for arr in self.pointcloud_by_zone.values():
            if len(arr) == 0:
                continue
            self.expected_num_points += len(arr)
            self.valid_points += int(np.isfinite(arr[:, :3]).all(axis=1).sum())

        return {
            "point_yield_rate": (
                self.valid_points / self.expected_num_points if self.expected_num_points > 0 else 0.0
            )
        }

    def shutdown(self) -> None:
        self.valid_points = 0
        self.expected_num_points = 0
