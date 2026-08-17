from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer

from lidar_zones.zones_api.profile_types import BaselineProfiles
from lidar_transforms.transforms import apply_transform, build_transform_matrix
from lidar_zones.zones_api import ZoneEngine


@dataclass
class FilterResult:
    """Result of an ROI filter operation for a single zone (or union)."""

    success: bool
    message: str
    filtered_xyz: np.ndarray | None         # shape (N, 3) in map frame, or None on failure
    filtered_xyz_sensor: np.ndarray | None  # shape (N, 3) in original sensor frame
    intensities: np.ndarray | None          # shape (N,) or None if cloud has no intensity field
    has_intensity: bool = False


class ROIFilter:
    """Decodes a raw PointCloud2 and produces per-zone spatial + projective filtered results."""

    def __init__(self) -> None:
        # The engine wraps each ZoneBounds into its plugin to run geometry-specific masks.
        self._engine = ZoneEngine()

    def filter(
        self,
        msg: PointCloud2,
        tf_buffer: Buffer,
        profiles: BaselineProfiles,
        y_padding: dict[str, float],
        z_padding: dict[str, float],
        tf_timeout_sec: float = 0.05,
    ) -> dict[str, dict[str, FilterResult]]:
        """Decode and transform the cloud once, then run both filters per zone.

        All preflight + decode work (field check, TF lookup, point decode, map-frame
        transform) happens here exactly once. The two filter methods receive the
        pre-decoded arrays and only apply their respective per-zone masks.

        Args:
            msg: Raw PointCloud2 message (any sensor frame).
            tf_buffer: TF2 buffer to look up the sensor→map transform.
            profiles: Built baseline profiles containing the per-zone filters.
            tf_timeout_sec: Timeout for the TF lookup.

        Returns:
            Nested dict keyed first by zone name, then by filter type:
              {
                'zone_a': {'spatial_cloud': FilterResult, 'projective_cloud': FilterResult},
                'zone_b': {'spatial_cloud': FilterResult, 'projective_cloud': FilterResult},
                ...
              }
            On a shared preflight failure every zone is populated with the same
            failure FilterResult under both keys — callers can iterate uniformly
            without special-casing the global-failure path.
        """
        field_names = [f.name for f in msg.fields]

        if 'x' not in field_names or 'y' not in field_names or 'z' not in field_names:
            return self._preflight_failure_dict(profiles, 'Cloud is missing XYZ fields')

        has_intensity = 'intensity' in field_names

        transform_4x4 = self._lookup_transform(msg, tf_buffer, tf_timeout_sec)
        if transform_4x4 is None:
            return self._preflight_failure_dict(
                profiles, f'TF lookup failed: {msg.header.frame_id} → map'
            )

        read_fields = ['x', 'y', 'z', 'intensity'] if has_intensity else ['x', 'y', 'z']
        raw_points = list(point_cloud2.read_points(msg, field_names=read_fields, skip_nans=True))

        if not raw_points:
            return self._preflight_failure_dict(profiles, 'Cloud is empty after NaN removal')

        arr = np.array(raw_points)
        xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1).astype(np.float64)
        intensities = arr['intensity'].astype(np.float32) if has_intensity else None

        # DIAGNOSTIC — count zero / near-zero returns coming from the sensor
        norms = np.linalg.norm(xyz, axis=1)
        n_zero = int((norms < 0.01).sum())
        print(f'[ROIFilter] raw n={len(xyz)} near-zero(<1cm)={n_zero}', flush=True)

        # Drop invalid sensor returns (Orbbec reports no-confidence pixels as ~0).
        # Done in sensor frame so the threshold is meaningful regardless of TF.
        valid = norms > 0.05
        xyz = xyz[valid]
        if intensities is not None:
            intensities = intensities[valid]

        xyz_map = apply_transform(xyz, transform_4x4)
        total_points = len(xyz)

        # Lidar's CURRENT position in the map frame = translation of the sensor→map
        # transform looked up this scan. The projective frustum is the cone from the
        # lidar's viewpoint, so it must be measured from this live apex rather than
        # the frozen profiles.lidar_position — otherwise the cone stays anchored to
        # wherever the lidar sat when profiles were built. Points and cone thus share
        # one TF snapshot. (The spatial box needs no apex, so it ignores this.)
        lidar_position_map = transform_4x4[:3, 3]

        spatial_per_zone = self.spatial_filter(xyz, xyz_map, intensities, has_intensity, total_points, profiles)
        projective_per_zone = self.projective_frustrum_filter(
            xyz, xyz_map, intensities, has_intensity, total_points, profiles,
            y_padding, z_padding, lidar_position_map,
        )

        return {
            zb.name: {
                'spatial_cloud': spatial_per_zone[zb.name],
                'projective_cloud': projective_per_zone[zb.name],
            }
            for zb in profiles.zone_bounds
        }

    def spatial_filter(
        self,
        xyz: np.ndarray,
        xyz_map: np.ndarray,
        intensities: np.ndarray | None,
        has_intensity: bool,
        total_points: int,
        profiles: BaselineProfiles,
    ) -> dict[str, FilterResult]:
        """Apply the per-zone 3D bounding-box mask to a pre-decoded cloud.

        Returns one FilterResult per zone. A zone with zero points in its bbox
        returns success=False with an informational message — that's a valid
        empty zone, not a filter failure.
        """
        per_zone_masks = self._spatial_mask(xyz_map, profiles)
        return self._build_per_zone_results(
            per_zone_masks, xyz, xyz_map, intensities, has_intensity, total_points, 'spatial bbox'
        )

    def projective_frustrum_filter(
        self,
        xyz: np.ndarray,
        xyz_map: np.ndarray,
        intensities: np.ndarray | None,
        has_intensity: bool,
        total_points: int,
        profiles: BaselineProfiles,
        y_padding: dict[str, float],
        z_padding: dict[str, float],
        lidar_position: np.ndarray | None = None,
    ) -> dict[str, FilterResult]:
        """Apply the per-zone angular (azimuth/elevation) mask to a pre-decoded cloud.

        Angular bounds are derived per-frame from each zone's 4 planar corners
        with `y_padding` / `z_padding` applied INWARD — corners shrink toward the
        zone center, producing a tighter cone that excludes near-edge clutter.
        Range is not tested; the cone extends to infinity. Returns one
        FilterResult per zone.

        Args:
            y_padding: {zone_name: y_pad_meters}. Lookup is by zone name; zones
                missing from the dict default to 0.0.
            z_padding: {zone_name: z_pad_meters}. Same shape.
            lidar_position: Live lidar apex in the map frame (this scan). Falls
                back to the frozen profiles.lidar_position when not supplied.
        """
        per_zone_masks = self._frustrum_mask(xyz_map, profiles, y_padding, z_padding, lidar_position)
        return self._build_per_zone_results(
            per_zone_masks, xyz, xyz_map, intensities, has_intensity, total_points, 'projective frustrum'
        )

    def union_result(self, per_zone_results: dict[str, FilterResult]) -> FilterResult:
        """Aggregate per-zone results into a single union FilterResult.

        Useful for publishing/visualization where zone identity isn't needed.
        Zones whose individual results failed (e.g. zero points) contribute
        nothing. Returns success=False if no zone succeeded.

        Overlap policy: a point that passed multiple zones' masks appears once
        per zone in the union (duplicates intentional — matches the per-zone
        dict's duplicate-on-overlap policy).
        """
        successful = [r for r in per_zone_results.values() if r.success]

        if not successful:
            return FilterResult(
                success=False,
                message='No zone produced any points',
                filtered_xyz=None,
                filtered_xyz_sensor=None,
                intensities=None,
            )

        has_intensity = successful[0].has_intensity
        filtered_xyz = np.vstack([r.filtered_xyz for r in successful])
        filtered_xyz_sensor = np.vstack([r.filtered_xyz_sensor for r in successful])
        intensities = (
            np.concatenate([r.intensities for r in successful]) if has_intensity else None
        )

        return FilterResult(
            success=True,
            message=f'Union of {len(successful)} zones, {filtered_xyz.shape[0]} points',
            filtered_xyz=filtered_xyz,
            filtered_xyz_sensor=filtered_xyz_sensor,
            intensities=intensities,
            has_intensity=has_intensity,
        )

    def to_pointcloud2(
        self,
        result: FilterResult,
        stamp,
        frame_id: str = 'rslidar',
        use_map_frame: bool = False,
    ) -> PointCloud2:
        """Pack a (single or union) FilterResult into a PointCloud2 message.

        Args:
            result: A successful FilterResult (result.success must be True).
            stamp: ROS timestamp for the output cloud header.
            frame_id: Frame ID for the output cloud (used when use_map_frame is False).
            use_map_frame: If True, use map-frame coordinates; otherwise sensor-frame.

        Returns:
            PointCloud2 message with XYZ (+ intensity if available).
        """
        assert result.success and result.filtered_xyz_sensor is not None

        xyz = result.filtered_xyz if use_map_frame else result.filtered_xyz_sensor
        actual_frame_id = 'map' if use_map_frame else frame_id

        return self._pack_cloud(xyz, result.intensities, result.has_intensity, stamp, actual_frame_id)

    def _pack_cloud(self, xyz, intensities, has_intensity, stamp, frame_id) -> PointCloud2:
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        xyz = xyz.astype(np.float32)

        if has_intensity and intensities is not None:
            fields.append(PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1))
            cloud_points = np.hstack([xyz, intensities.reshape(-1, 1).astype(np.float32)])
        else:
            cloud_points = xyz

        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        return point_cloud2.create_cloud(header, fields, cloud_points.tolist())

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _lookup_transform(
        self,
        msg: PointCloud2,
        tf_buffer: Buffer,
        timeout_sec: float,
    ) -> np.ndarray | None:
        try:
            tf_stamped = tf_buffer.lookup_transform(
                'map', msg.header.frame_id, Time(), Duration(seconds=timeout_sec)
            )
        except Exception:
            return None

        return build_transform_matrix(
            tf_stamped.transform.translation,
            tf_stamped.transform.rotation,
        )

    def _spatial_mask(self, xyz_map: np.ndarray, profiles: BaselineProfiles) -> dict[str, np.ndarray]:
        """Per-zone boolean masks for the spatial filter.

        Routes each zone to its geometry-specific spatial mask via the
        ZoneEngine (planar bbox / cylindrical radial shell, both over the full
        extent). A point can appear in multiple zones if they overlap — no
        exclusivity is enforced.
        """
        return {
            zb.name: self._engine.wrap(zb).spatial_mask(xyz_map)
            for zb in profiles.zone_bounds
        }

    def _frustrum_mask(
        self,
        xyz_map: np.ndarray,
        profiles: BaselineProfiles,
        y_padding: dict[str, float],
        z_padding: dict[str, float],
        lidar_position: np.ndarray | None = None,
    ) -> dict[str, np.ndarray]:
        """Per-zone boolean masks for the projective (angular) filter.

        Precomputes each point's (azimuth, elevation) relative to the lidar once,
        then routes every zone to its geometry-specific angular mask via the
        ZoneEngine. Each plugin applies its padding INWARD to tighten the cone.
        A point can appear in multiple zones if windows overlap.

        The bearing apex is the CURRENT lidar position (`lidar_position`, from the
        live sensor→map transform) so the cone tracks the moving lidar. Both the
        point bearings and the zone-corner bearings (in the handlers) are measured
        from this same apex. Falls back to the frozen profiles.lidar_position only
        if no live apex is supplied.
        """
        apex = profiles.lidar_position if lidar_position is None else lidar_position
        lx, ly, lz = apex
        dx = xyz_map[:, 0] - lx
        dy = xyz_map[:, 1] - ly
        dz = xyz_map[:, 2] - lz
        az = np.arctan2(dy, dx)
        el = np.arctan2(dz, np.sqrt(dx ** 2 + dy ** 2))

        return {
            zb.name: self._engine.wrap(zb).projective_mask(az, el, apex, y_padding, z_padding)
            for zb in profiles.zone_bounds
        }

    def _build_per_zone_results(
        self,
        per_zone_masks: dict[str, np.ndarray],
        xyz: np.ndarray,
        xyz_map: np.ndarray,
        intensities: np.ndarray | None,
        has_intensity: bool,
        total_points: int,
        filter_label: str,
    ) -> dict[str, FilterResult]:
        """Build a per-zone FilterResult dict by applying each zone's mask."""
        results: dict[str, FilterResult] = {}
        for zone_name, mask in per_zone_masks.items():
            filtered_xyz = xyz_map[mask]
            if filtered_xyz.shape[0] == 0:
                results[zone_name] = FilterResult(
                    success=False,
                    message=f'No points in {filter_label} for zone {zone_name}',
                    filtered_xyz=None,
                    filtered_xyz_sensor=None,
                    intensities=None,
                )
                continue
            results[zone_name] = FilterResult(
                success=True,
                message=f'Filtered {filtered_xyz.shape[0]} of {total_points} points',
                filtered_xyz=filtered_xyz,
                filtered_xyz_sensor=xyz[mask],
                intensities=intensities[mask] if intensities is not None else None,
                has_intensity=has_intensity,
            )
        return results

    def _preflight_failure_dict(
        self,
        profiles: BaselineProfiles,
        message: str,
    ) -> dict[str, dict[str, FilterResult]]:
        """Build a per-zone dict where every entry holds the same shared failure FilterResult."""
        failure = FilterResult(
            success=False,
            message=message,
            filtered_xyz=None,
            filtered_xyz_sensor=None,
            intensities=None,
        )
        return {
            zb.name: {'spatial_cloud': failure, 'projective_cloud': failure}
            for zb in profiles.zone_bounds
        }
