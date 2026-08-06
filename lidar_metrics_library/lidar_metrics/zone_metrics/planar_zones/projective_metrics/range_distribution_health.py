from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
import numpy as np
import math


class RangeDistributionHealth(MetricsBase):
    """Per-zone 3D range-error distribution against the expected zone surface.

    For each return, the ray direction from the lidar origin is intersected with
    the zone's planar surface (the plane x = x_surface) to find the expected hit
    point — where that ray should have landed. The 3D distance between the actual
    point and that expected on-surface point is the range error. Because the
    actual point and the expected hit lie on the same ray from the origin, this
    distance equals the along-ray difference between the measured range and the
    expected range to the surface — i.e. a true per-beam ranging error. Each error
    is also expressed as a percentage of the expected ray distance (the distance
    from the origin to the expected hit point), so a fixed miss counts for more on
    a near surface than a far one.

    The cloud is already frustum-filtered per zone, so every point's ray passes
    through the zone; no range gating is applied, so gross outliers are kept and
    surface in the worst-error figure. Errors are pooled per zone (not across
    zones) so the report attributes each distribution to its own zone via the
    "<zone>_range_error_*" key prefix.
    """

    def __init__(self, pointcloud_by_zone, profiles=None, baseline_profiles=None):
        self.errors_by_zone: dict[str, list[float]] = {}
        # Same errors expressed as a percentage of the expected ray distance.
        self.error_pcts_by_zone: dict[str, list[float]] = {}
        # Per-scan stat snapshots (raw + pct), averaged across scans in compute().
        self.scan_stats_by_zone: dict[str, list[dict]] = {}
        self.scan_stats_pct_by_zone: dict[str, list[dict]] = {}
        # Running top-K worst points (by error) across the run, with map-frame xyz.
        # Kept spatially distinct so the same static return isn't reported K times.
        self._n_worst: int = 0
        self._worst_min_sep_m: float = 0.05
        self._worst_by_zone: dict[str, list[tuple]] = {}
        self.x_surface_by_zone: dict[str, float] = {}
        super().__init__(pointcloud_by_zone, profiles, baseline_profiles)

    def setup(self) -> None:
        params = self.config['lidar_metrics_parameters'].get('range_distribution_health', {})
        self._n_worst = int(params.get('number_of_worst_points', 0))
        self._worst_min_sep_m = float(params.get('worst_point_min_separation_m', 0.05))
        if self.profiles is not None:
            self.x_surface_by_zone = {
                zb.name: float(zb.x_surface) for zb in self.profiles.zone_bounds
            }

    def update(self, pointcloud_by_zone) -> None:
        self.pointcloud_by_zone = pointcloud_by_zone
        if self.profiles is None:
            return  # need zone geometry (x_surface) to compute expected ranges

        lidar_pos = self.profiles.lidar_position

        for zone, arr in pointcloud_by_zone.items():
            if len(arr) == 0:
                continue
            finite = arr[np.isfinite(arr[:, :3]).all(axis=1)]
            if len(finite) == 0:
                continue
            x_surface = self.x_surface_by_zone.get(zone)
            if x_surface is None:
                continue

            # Ray vector origin -> point, and the measured range along it.
            dx = finite[:, 0] - lidar_pos[0]
            dy = finite[:, 1] - lidar_pos[1]
            dz = finite[:, 2] - lidar_pos[2]
            range_actual = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            # Expected hit = ray-plane intersection at x = x_surface. With the ray
            # parameterized as L + s*(P - L), s solves L_x + s*dx = x_surface. The
            # actual point P sits at s = 1, so the 3D error is |s - 1| * range, and
            # the expected ray distance (origin -> expected hit) is s * range.
            s = (x_surface - lidar_pos[0]) / dx
            errors = np.abs(s - 1.0) * range_actual
            expected_dist = s * range_actual
            errors_pct = np.where(expected_dist > 0.0, errors / expected_dist * 100.0, 0.0)

            self.errors_by_zone.setdefault(zone, []).extend(errors.tolist())
            self.error_pcts_by_zone.setdefault(zone, []).extend(errors_pct.tolist())
            self.scan_stats_by_zone.setdefault(zone, []).append(self._scan_stats(errors))
            self.scan_stats_pct_by_zone.setdefault(zone, []).append(self._scan_stats(errors_pct))
            self._update_worst(zone, errors, finite[:, :3])

    def compute(self) -> dict[str, float]:
        result: dict[str, float] = {}

        # Report every zone we know about (from profiles), even ones with no
        # returns this run, so the per-zone schema is stable.
        zones = self.x_surface_by_zone.keys() or self.errors_by_zone.keys()
        for zone in zones:
            errors = sorted(self.errors_by_zone.get(zone, []))
            errors_pct = sorted(self.error_pcts_by_zone.get(zone, []))

            result.update(self._distribution_stats(errors, f'{zone}_range_error'))
            result.update(self._distribution_stats(errors_pct, f'{zone}_range_error_pct'))
            result.update(self._reduce_scan_stats(self.scan_stats_by_zone.get(zone, []), f'{zone}_range_error_per_scan'))
            result.update(self._reduce_scan_stats(self.scan_stats_pct_by_zone.get(zone, []), f'{zone}_range_error_pct_per_scan'))
            result[f'{zone}_worst_range_error'] = float(errors[-1]) if errors else 0.0
            result[f'{zone}_worst_range_error_pct'] = float(errors_pct[-1]) if errors_pct else 0.0
            for n, (err, x, y, z) in enumerate(self._worst_by_zone.get(zone, [])):
                result[f'{zone}_worst_point_{n}_x'] = x
                result[f'{zone}_worst_point_{n}_y'] = y
                result[f'{zone}_worst_point_{n}_z'] = z
                result[f'{zone}_worst_point_{n}_error'] = err

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
        self.errors_by_zone.clear()
        self.error_pcts_by_zone.clear()
        self.scan_stats_by_zone.clear()
        self.scan_stats_pct_by_zone.clear()
        self._worst_by_zone.clear()
