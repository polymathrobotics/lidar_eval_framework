from pathlib import Path

import yaml
from urdf_parser_py.urdf import Box, Color, Joint, Link, LinkMaterial, Pose, Robot, Visual, Cylinder

from lidar_zones.zones_api import ZoneEngine


class URDFGenerator:

    def __init__(self, environment_config_dict: dict, lidar_config_dict: dict):
        self.environment_config_dict = environment_config_dict
        self.lidar_config_dict = lidar_config_dict
        self.bench_urdf = Robot(name='lidar_bench')
        # Each zone's URDF link is built by its geometry plugin (construct_urdf_link),
        # routed via the same ZoneEngine registry the runtime uses.
        self._engine = ZoneEngine()

        cart_defaults_path = Path(__file__).parent / 'config' / 'cart_defaults.yaml'
        with cart_defaults_path.open('r') as f:
            self._cart_defaults = yaml.safe_load(f)

    def build_harness(self, output_urdf_path: Path) -> None:
        self.build_base_environment()
        self.build_environment_config()
        self.build_lidar_config()

        output_urdf_path.parent.mkdir(parents=True, exist_ok=True)
        output_urdf_path.write_text(self.bench_urdf.to_xml_string())



    def build_base_environment(self) -> None:
        links = self._cart_defaults['links']
        joints = self._cart_defaults['joints']

        for link_cfg in links.values():
            box_size = link_cfg.get('box_size')
            self.bench_urdf.add_link(self.build_link(
                name=link_cfg['name'],
                geometry=Box(size=box_size) if box_size is not None else None,
                color_rgba=link_cfg.get('color_rgba'),
            ))

        for joint_name, cfg in joints.items():
            self.bench_urdf.add_joint(self.build_joint(
                name=joint_name,
                parent=cfg['parent'],
                child=cfg['child'],
                joint_type=cfg['type'],
                origin_xyz=cfg['xyz'],
                origin_rpy=cfg['rpy'],
                axis=cfg.get('axis'),
            ))

    def build_lidar_config(self):
        lidar_params = self.lidar_config_dict.get('lidar', {})
        lidar_frame = lidar_params.get('frame')
        joint_rpy = lidar_params.get('joint_rpy_rad')
        map_to_motor_height_m = lidar_params.get('map_to_motor_height_m')
        motor_to_lidar_height_m = lidar_params.get('motor_to_lidar_height_m')

        # map -> motor_axis: vertical lift from the ground to the rotating motor
        # axis (spins about z). This replaces the old cart chain.
        self.bench_urdf.add_joint(self.build_joint(
            name='map_to_motor',
            parent='map',
            child='motor_axis',
            joint_type='continuous',
            origin_xyz=[0.0, 0.0, map_to_motor_height_m],
            origin_rpy=[0.0, 0.0, 0.0],
            axis=[0, 0, 1],
        ))

        # motor_axis -> lidar: sensor mount offset + orientation.
        self.bench_urdf.add_link(self.build_link(name=lidar_frame))
        self.bench_urdf.add_joint(self.build_joint(
            name=f'{lidar_frame}_joint',
            parent='motor_axis',
            child=lidar_frame,
            joint_type='fixed',
            origin_xyz=[0.0, 0.0, motor_to_lidar_height_m],
            origin_rpy=joint_rpy,
        ))



    def build_environment_config(self):
        zones = self.environment_config_dict.get('zones', [])
        zone_properties = self.environment_config_dict.get('zone_properties', {})
        zone_joints = self.environment_config_dict.get('zone_joints', {})
        world_placement = self.environment_config_dict.get('world_placement', {})

        # Step 1: build links for all zones — each geometry's link is built by its
        # plugin. `name` falls back to zone_name when the props omit an explicit
        # `frame`, matching the plugin's frame→name→default lookup.
        for zone_name in zones:
            props = zone_properties.get(zone_name, {})
            zone_type = props.get('type', 'planar')
            link = self._engine.plugin_for(zone_type).construct_urdf_link({'name': zone_name, **props})
            self.bench_urdf.add_link(link)

        # Step 2: build joint from map to anchor zone
        anchor_zone = world_placement.get('child_zone')
        anchor_props = zone_properties.get(anchor_zone, {})
        anchor_z_offset = anchor_props.get('z_offset', 0.0)
        anchor_height = anchor_props.get('height', 0.0)
        world_x = world_placement.get('x_offset') or 0.0
        world_y = world_placement.get('y_offset') or 0.0
        self.bench_urdf.add_joint(self.build_joint(
            name=f'map_to_{anchor_zone}',
            parent='map',
            child=anchor_props.get('frame', anchor_zone),
            joint_type='fixed',
            origin_xyz=[world_x, world_y, anchor_z_offset + anchor_height / 2.0],
            origin_rpy=[0.0, 0.0, 0.0],
        ))

        # Step 3: BFS from anchor zone to build joints for all other zones
        queue = [(anchor_zone, 0.0)]
        visited = {anchor_zone}

        while queue:
            current_zone, current_y = queue.pop(0)
            current_length = zone_properties.get(current_zone, {}).get('length', 0.0)
            neighbors = zone_joints.get(current_zone, {})

            for side in ('left_zone', 'right_zone'):
                joint_cfg = neighbors.get(side)
                if joint_cfg is None:
                    continue
                neighbor_zone = joint_cfg.get('zone')
                if neighbor_zone is None or neighbor_zone in visited:
                    continue
                visited.add(neighbor_zone)

                x_offset = joint_cfg.get('x_offset', 0.0)
                y_offset = joint_cfg.get('y_offset', 0.0)
                neighbor_props = zone_properties.get(neighbor_zone, {})
                neighbor_length = neighbor_props.get('length', 0.0)
                sign = -1.0 if side == 'right_zone' else 1.0
                neighbor_y = current_y + sign * (current_length / 2.0 + neighbor_length / 2.0 + y_offset)

                z_delta = (neighbor_props.get('z_offset', 0.0) + neighbor_props.get('height', 0.0) / 2.0) - (anchor_z_offset + anchor_height / 2.0)
                self.bench_urdf.add_joint(self.build_joint(
                    name=f'{anchor_zone}_to_{neighbor_zone}',
                    parent=anchor_props.get('frame', anchor_zone),
                    child=neighbor_props.get('frame', neighbor_zone),
                    joint_type='fixed',
                    origin_xyz=[x_offset, neighbor_y, z_delta],
                    origin_rpy=[0.0, 0.0, 0.0],
                ))
                queue.append((neighbor_zone, neighbor_y))

    def build_joint(
        self,
        name: str,
        parent: str,
        child: str,
        joint_type: str = 'fixed',
        origin_xyz: list[float] | None = None,
        origin_rpy: list[float] | None = None,
        axis: list[float] | None = None,
    ) -> Joint:
        return Joint(
            name=name,
            parent=parent,
            child=child,
            joint_type=joint_type,
            axis=axis,
            origin=Pose(
                xyz=origin_xyz or [0.0, 0.0, 0.0],
                rpy=origin_rpy or [0.0, 0.0, 0.0],
            ),
        )

    def build_link(
        self,
        name: str,
        geometry: Box | Cylinder | None = None,
        color_rgba: list[float] | None = None,
    ) -> Link:
        link = Link(name=name)
        if geometry is not None:
            color = Color()
            color.rgba = color_rgba or [0.5, 0.5, 0.5, 1.0]
            material = LinkMaterial(name=f'{name}_material')
            material.color = color
            visual = Visual()
            visual.geometry = geometry
            visual.material = material
            link.add_aggregate('visual', visual)
        return link
