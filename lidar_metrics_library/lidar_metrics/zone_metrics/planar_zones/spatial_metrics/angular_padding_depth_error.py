from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np


class AngularPaddingDepthError(MetricsBase):
    """For each zone, look at points whose rays fall in a padding ring around the
    zone's angular window and report their depth error against the expected surface.

    KNOWN LIMITATION: under the per-zone refactor this metric only receives the
    in-window points (one entry per zone). The "padding ring" by definition lives
    OUTSIDE any zone's angular window, so the data this metric needs is no longer
    delivered through the standard engine pipeline. compute() returns NaN for all
    zones until the engine grows a "raw cloud" path or this metric is rewritten
    against a wider angular window.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)
        self.padding_rad = None
        self.depth_tolerance_pct = None

    def setup(self) -> None:
        params = self.config['lidar_metrics_parameters']['angular_padding_depth_error']
        self.padding_rad = float(params['padding_rad'])
        self.depth_tolerance_pct = float(params['depth_tolerance_pct'])

    def update(self, pointcloud_by_zone) -> None:
        self.pointcloud_by_zone = pointcloud_by_zone

    def compute(self) -> dict[str, float]:
        # No access to padding-ring points under the per-zone dict contract.
        # See the class docstring for the design limitation.
        if self.profiles is None:
            return {}
        return {f'{zb.name}_padding_depth_error_pct': float('nan') for zb in self.profiles.zone_bounds}

    def shutdown(self) -> None:
        return
