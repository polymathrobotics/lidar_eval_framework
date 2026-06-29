import math
from collections import deque


def PlanarZoneOrchestrator(props: dict) -> dict:
    """Build the planar-specific ROI fields (width / z_bounds / y,z padding)."""
    z_offset = props.get('z_offset', 0.0)
    height = props.get('height', 0.0)
    return {
        'width': props.get('length', 0.0),
        'z_bounds': [z_offset, round(z_offset + height, 6)],
        'y_padding': float(props.get('y_padding', 0.0)),
        'z_padding': float(props.get('z_padding', 0.0)),
    }


def CylindricalZoneOrchestrator(props: dict) -> dict:
    """Build the cylindrical-specific ROI fields (radius / height / paddings).

    Padding falls back to the planar y/z padding keys so environment configs
    that predate the radius/height padding split still produce sensible values
    (radial ← y, axial ← z).
    """
    return {
        'height': float(props.get('height', 0.0)),
        'radius': float(props.get('radius', 0.0)),
        'radius_padding': float(props.get('radius_padding', props.get('y_padding', 0.0))),
        'height_padding': float(props.get('height_padding', props.get('z_padding', 0.0))),
        'outward_radius_padding': float(props.get('outward_radius_padding')),
    }


# Maps a zone's geometry type to the orchestrator that builds its ROI fields.
# Unknown types fall back to the planar orchestrator.
ZONE_ORCHESTRATOR_MAP = {
    'planar': PlanarZoneOrchestrator,
    'cylindrical': CylindricalZoneOrchestrator,
}


def build_zone_roi_fields(zone_type: str, props: dict) -> dict:
    """Dispatch to the geometry-specific ROI field orchestrator for `zone_type`."""
    orchestrator = ZONE_ORCHESTRATOR_MAP.get(zone_type, PlanarZoneOrchestrator)
    return orchestrator(props)


class PolysetupUtils():

    @staticmethod
    def compute_panning_angles(
        horizontal_fov_deg: float,
        environment_config: dict,
        breathing_room_m: float = 1.0,
    ) -> list[float]:
        """Compute 5 pan angles spanning the lidar's coverage range for a zone region.

        The lidar sits at the world origin and pans about +Z (motor_joint in
        lidar_bench.urdf). We resolve each zone's center via world_placement
        and zone_joints, expand to corners with length/2, then inflate the
        bounding region by breathing_room_m on both X (pull the near face
        toward the lidar) and Y (widen the lateral extents). The pan extremes
        are solved algebraically: each FOV edge just kisses an inflated corner.

        Center is hard-pinned to 0.0 (lidar looks straight down +X), and the
        intermediate mids are taken halfway between center and each extreme.
        Every value is truncated toward zero (positives floored, negatives
        ceiled) so the output matches the integer-valued double convention the
        automation manager has historically used.

        Args:
            horizontal_fov_deg: lidar horizontal FOV in degrees.
            environment_config: parsed environment yaml dict.
            breathing_room_m: linear padding in meters applied to the zone AABB
                on both X (near face pulled toward the lidar) and Y (lateral
                extents widened). Larger values reserve more FOV slack as
                buffer between the arc edge and the zones, which tightens the
                pan sweep range — at the limit the sweep collapses to a single
                centered pose with the FOV just enclosing the padded region.

        Returns:
            List of 5 doubles in degrees, ordered left-to-right (descending θ):
            [left_extreme, left_mid, 0.0, right_mid, right_extreme]. Positive =
            pan toward +Y per the motor_joint Z axis.
        """
        zone_centers = PolysetupUtils._resolve_zone_centers(environment_config)
        zone_properties = environment_config.get('zone_properties', {})

        x_min = math.inf
        y_min = math.inf
        y_max = -math.inf
        for zone_name, (cx, cy) in zone_centers.items():
            length = float(zone_properties.get(zone_name, {}).get('length', 0.0))
            x_min = min(x_min, cx)
            y_min = min(y_min, cy - length / 2.0)
            y_max = max(y_max, cy + length / 2.0)

        x_near = x_min - breathing_room_m
        if 0.0 >= x_near:
            raise ValueError(
                f'breathing_room_m={breathing_room_m} pulls near face to x={x_near:.3f} m; '
                f'must stay positive (lidar sits at origin).'
            )

        alpha_left = math.atan2(y_max + breathing_room_m, x_near)
        alpha_right = math.atan2(y_min - breathing_room_m, x_near)
        region_span = alpha_left - alpha_right

        fov_rad = math.radians(horizontal_fov_deg)
        if region_span > fov_rad:
            raise ValueError(
                f'Inflated zone region subtends {math.degrees(region_span):.1f} deg, '
                f'exceeding FOV {horizontal_fov_deg:.1f} deg — no single pan angle '
                f'can cover it. Reduce breathing_room_m or use a wider-FOV lidar.'
            )

        half_fov = fov_rad / 2.0
        theta_left = math.degrees(alpha_right + half_fov)
        theta_right = math.degrees(alpha_left - half_fov)
        theta_left_mid = theta_left / 2.0
        theta_right_mid = theta_right / 2.0

        # math.trunc rounds toward zero: positives floored, negatives ceiled.
        return [
            float(math.trunc(theta_left)),
            float(math.trunc(theta_left_mid)),
            0.0,
            float(math.trunc(theta_right_mid)),
            float(math.trunc(theta_right)),
        ]

    @staticmethod
    def _resolve_zone_centers(environment_config: dict) -> dict[str, tuple[float, float]]:
        """BFS from world_placement.child_zone through zone_joints to resolve
        each zone's center (x, y) in the lidar/map frame.

        Matches HarnessBuilderInterface.build_environment_config: left_zone
        advances +Y, right_zone advances -Y, and y_offset is an edge-to-edge
        gap (so neighbor_y = parent_y + sign*(parent_length/2 + child_length/2
        + y_offset)).
        """
        zones = environment_config.get('zones', [])
        zone_properties = environment_config.get('zone_properties', {})
        zone_joints = environment_config.get('zone_joints', {})
        world_placement = environment_config.get('world_placement', {})

        anchor = world_placement.get('child_zone')
        if anchor is None or anchor not in zones:
            raise ValueError(
                f'world_placement.child_zone "{anchor}" missing from zones list {zones}.'
            )

        centers: dict[str, tuple[float, float]] = {
            anchor: (
                float(world_placement.get('x_offset') or 0.0),
                float(world_placement.get('y_offset') or 0.0),
            ),
        }

        queue = deque([anchor])
        while queue:
            parent = queue.popleft()
            parent_x, parent_y = centers[parent]
            parent_length = float(zone_properties.get(parent, {}).get('length', 0.0))
            for side, sign in (('left_zone', 1.0), ('right_zone', -1.0)):
                joint = zone_joints.get(parent, {}).get(side)
                if joint is None:
                    continue
                child = joint.get('zone')
                if child is None or child in centers:
                    continue
                child_length = float(zone_properties.get(child, {}).get('length', 0.0))
                child_x = parent_x + float(joint.get('x_offset') or 0.0)
                child_y = parent_y + sign * (
                    parent_length / 2.0
                    + child_length / 2.0
                    + float(joint.get('y_offset') or 0.0)
                )
                centers[child] = (child_x, child_y)
                queue.append(child)

        missing = [z for z in zones if z not in centers]
        if missing:
            raise ValueError(
                f'Zones not reachable from anchor "{anchor}" via zone_joints: {missing}.'
            )

        return centers
