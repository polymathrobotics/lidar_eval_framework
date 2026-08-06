from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np
import math


class ZoneSurfaceDepthError(MetricsBase):
    """Per-zone depth-error distribution against TF ground truth.

    For each zone, computes the 1D difference between each point's actual X
    distance from the lidar and the expected depth stored in
    ZoneBounds.expected_depth_m (no y/z or ray geometry — purely the depth axis).
    The absolute depth error is pooled per zone over the whole run and reduced to
    a percentile distribution (min/p10/p50/p90/p99/max, mean, std) plus the worst
    error, mirroring RangeDistributionHealth. Each error is also expressed as a
    percentage of the zone's expected depth so a fixed miss counts for more on a
    near surface than a far one.

    Errors are pooled per zone (not across zones) so the report attributes each
    distribution to its own zone via the "<zone>_depth_error_*" key prefix.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        # Per-zone absolute depth errors over the union of all scans. Percentiles
        # need the samples, so unlike a streaming mean/std we keep them.
        self._zone_abs_errors: dict[str, list[float]] = {}
        # Per-scan stat snapshots (raw + pct), averaged across scans in compute().
        self._scan_stats_by_zone: dict[str, list[dict]] = {}
        self._scan_stats_pct_by_zone: dict[str, list[dict]] = {}
        # Running top-K worst points (by error) across the run, with map-frame xyz.
        # Kept spatially distinct so the same static return isn't reported K times.
        self._n_worst: int = 0
        self._worst_min_sep_m: float = 0.05
        self._worst_by_zone: dict[str, list[tuple]] = {}
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        params = self.config['lidar_metrics_parameters'].get('zone_surface_depth_error', {})
        self._n_worst = int(params.get('number_of_worst_points', 0))
        self._worst_min_sep_m = float(params.get('worst_point_min_separation_m', 0.05))
        if self.profiles is None:
            return
        for zb in self.profiles.zone_bounds:
            self._zone_abs_errors[zb.name] = []
            self._scan_stats_by_zone[zb.name] = []
            self._scan_stats_pct_by_zone[zb.name] = []

    def update(self, pointcloud_by_zone) -> None:
        if self.profiles is None:
            return
        self.pointcloud_by_zone = pointcloud_by_zone
        lidar_x = float(self.profiles.lidar_position[0])

        for zb in self.profiles.zone_bounds:
            zone_pts = pointcloud_by_zone.get(zb.name)
            if zone_pts is None or len(zone_pts) == 0:
                continue
            zone_pts = zone_pts[np.isfinite(zone_pts[:, :3]).all(axis=1)]
            if len(zone_pts) == 0:
                continue

            error = (zone_pts[:, 0] - lidar_x) - zb.expected_depth_m
            abs_err = np.abs(error)
            self._zone_abs_errors[zb.name].extend(abs_err.tolist())
            pct_scale = 100.0 / zb.expected_depth_m if zb.expected_depth_m != 0.0 else 0.0
            self._scan_stats_by_zone[zb.name].append(self._scan_stats(abs_err))
            self._scan_stats_pct_by_zone[zb.name].append(self._scan_stats(abs_err * pct_scale))
            self._update_worst(zb.name, abs_err, zone_pts[:, :3])

    def compute(self) -> dict[str, float]:
        if self.profiles is None:
            return {}

        result: dict[str, float] = {}
        for zb in self.profiles.zone_bounds:
            abs_errors = sorted(self._zone_abs_errors.get(zb.name, []))
            pct_scale = 100.0 / zb.expected_depth_m if zb.expected_depth_m != 0.0 else 0.0
            abs_errors_pct = [e * pct_scale for e in abs_errors]  # stays sorted (scale >= 0)

            result.update(self._distribution_stats(abs_errors, f'{zb.name}_depth_error'))
            result.update(self._distribution_stats(abs_errors_pct, f'{zb.name}_depth_error_pct'))
            result.update(self._reduce_scan_stats(self._scan_stats_by_zone.get(zb.name, []), f'{zb.name}_depth_error_per_scan'))
            result.update(self._reduce_scan_stats(self._scan_stats_pct_by_zone.get(zb.name, []), f'{zb.name}_depth_error_pct_per_scan'))
            result[f'{zb.name}_worst_depth_error'] = float(abs_errors[-1]) if abs_errors else 0.0
            result[f'{zb.name}_worst_depth_error_pct'] = float(abs_errors_pct[-1]) if abs_errors_pct else 0.0
            for n, (err, x, y, z) in enumerate(self._worst_by_zone.get(zb.name, [])):
                result[f'{zb.name}_worst_point_{n}_x'] = x
                result[f'{zb.name}_worst_point_{n}_y'] = y
                result[f'{zb.name}_worst_point_{n}_z'] = z
                result[f'{zb.name}_worst_point_{n}_error'] = err

        return result

    def _distribution_stats(self, sorted_vals: list[float], prefix: str) -> dict[str, float]:
        if not sorted_vals:
            return {
                f'{prefix}_min': 0.0,
                f'{prefix}_p10': 0.0,
                f'{prefix}_p50': 0.0,
                f'{prefix}_p90': 0.0,
                f'{prefix}_p99': 0.0,
                f'{prefix}_max': 0.0,
                f'{prefix}_mean': 0.0,
                f'{prefix}_std': 0.0,
            }
        mean = float(sum(sorted_vals) / len(sorted_vals))
        var = sum((x - mean) ** 2 for x in sorted_vals) / len(sorted_vals)
        return {
            f'{prefix}_min': float(sorted_vals[0]),
            f'{prefix}_p10': self._pct(sorted_vals, 10.0),
            f'{prefix}_p50': self._pct(sorted_vals, 50.0),
            f'{prefix}_p90': self._pct(sorted_vals, 90.0),
            f'{prefix}_p99': self._pct(sorted_vals, 99.0),
            f'{prefix}_max': float(sorted_vals[-1]),
            f'{prefix}_mean': mean,
            f'{prefix}_std': float(math.sqrt(var)),
        }

    @staticmethod
    def _pct(sorted_vals: list[float], p: float) -> float:
        n = len(sorted_vals)
        if n == 0:
            return 0.0
        idx = int(round((p / 100.0) * (n - 1)))
        idx = max(0, min(idx, n - 1))
        return float(sorted_vals[idx])

    def _scan_stats(self, vals) -> dict[str, float]:
        """Percentile/mean/std for a single scan's errors. Stored per scan so
        compute() can average them across scans (a typical-scan view, robust to
        one-off outliers — distinct from the pooled-over-all-points view)."""
        s = np.sort(vals)
        return {
            'p10': self._pct(s, 10.0),
            'p50': self._pct(s, 50.0),
            'p90': self._pct(s, 90.0),
            'p99': self._pct(s, 99.0),
            'mean': float(np.mean(vals)),
            'std': float(np.std(vals)),
        }

    @staticmethod
    def _reduce_scan_stats(scans: list[dict], prefix: str) -> dict[str, float]:
        """Average each per-scan stat across all scans that had returns. Min/max
        are intentionally omitted — the pooled keys already carry the single
        global best/worst across the whole run."""
        keys = ('p10', 'p50', 'p90', 'p99', 'mean', 'std')
        if not scans:
            return {f'{prefix}_{k}': 0.0 for k in keys}
        n = len(scans)
        return {f'{prefix}_{k}': float(sum(sc[k] for sc in scans) / n) for k in keys}

    def _update_worst(self, zone, errs, pts) -> None:
        """Keep the running top-K worst points (by error) across all scans, with
        their map-frame xyz. A candidate within _worst_min_sep_m of an already-kept
        point is treated as the same location (keeping the higher error), so the K
        reported points are distinct spots — not the same static return repeated
        across scans."""
        if self._n_worst <= 0 or len(errs) == 0:
            return
        worst = self._worst_by_zone.setdefault(zone, [])
        for i in np.argsort(errs)[::-1]:  # highest error first
            err = float(errs[i])
            if len(worst) >= self._n_worst and err <= worst[-1][0]:
                break  # list full and nothing lower can improve it
            self._insert_distinct(worst, (err, float(pts[i, 0]), float(pts[i, 1]), float(pts[i, 2])))

    def _insert_distinct(self, worst, cand) -> None:
        err, x, y, z = cand
        sep2 = self._worst_min_sep_m ** 2
        for j, (e, px, py, pz) in enumerate(worst):
            if (x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2 <= sep2:
                if err > e:  # same spot — keep the worse one
                    worst[j] = cand
                    worst.sort(key=lambda t: t[0], reverse=True)
                return
        worst.append(cand)
        worst.sort(key=lambda t: t[0], reverse=True)
        del worst[self._n_worst:]

    def shutdown(self) -> None:
        for errors in self._zone_abs_errors.values():
            errors.clear()
        for scans in self._scan_stats_by_zone.values():
            scans.clear()
        for scans in self._scan_stats_pct_by_zone.values():
            scans.clear()
        self._worst_by_zone.clear()
