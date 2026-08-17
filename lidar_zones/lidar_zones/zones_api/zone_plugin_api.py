"""The zone-plugin contract.

One `ZoneTypePlugin` subclass per zone geometry is the *single* place that
defines everything about that geometry: how to parse it, build its resolved
bounds, mask point clouds against it, (de)serialize it, describe it for viz,
draw its RViz markers, and draw its URDF link. Add a geometry = add one subclass
and register it with a row in `zones_types_registry.yaml`; every consumer (zones
node, filter, controller) picks it up through the ZoneEngine with no further edits.

A plugin *instance* wraps a resolved `ZoneBounds` (`self.bounds`) and provides the
per-zone runtime operations. Construction (parse / build / from_dict) happens
before an instance exists, so those are classmethods.

Dependency rule: keep module-level imports light (numpy only). Heavy or ROS-ish
imports (urdf_parser_py, visualization_msgs) go INSIDE the one method that needs
them, so pure consumers like the filter never drag them in.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from lidar_zones.zones_api.profile_types import FramePose, ZoneBounds, ZoneConfig, ZoneType


class ZoneTypePlugin(ABC):

    
    @property
    @abstractmethod
    def zone_type_cls(self) -> type:
        """The ZoneType subclass (struct) this geometry defines."""

    @property
    @abstractmethod
    def bounds_cls(self) -> type:
        """The ZoneBounds subclass (struct) this geometry defines."""

    def __init__(self, bounds: ZoneBounds) -> None:
        self.bounds = bounds

    @property
    def name(self) -> str:
        return self.bounds.name

    # ---- construction: no instance yet -> class/static methods ---------------

    @classmethod
    @abstractmethod
    def parse_zone_type(cls, raw: dict, location: str) -> ZoneType:
        """Validate + parse this geometry's YAML fields into a ZoneType."""

    @classmethod
    @abstractmethod
    def build(cls, zone_cfg: ZoneConfig, pose: FramePose, lidar_pos: np.ndarray) -> "ZoneTypePlugin":
        """Resolve ZoneType + TF pose into bounds; return a plugin wrapping them."""

    @classmethod
    @abstractmethod
    def from_dict(cls, d: dict, zone_config: ZoneConfig) -> "ZoneTypePlugin":
        """Deserialize a bounds dict (from /get_profiles) into a plugin instance."""

    @classmethod
    @abstractmethod
    def zone_type_to_dict(cls, zone_type: ZoneType) -> dict:
        """Serialize a ZoneType's fields to a plain dict (no geometry tag — the
        ZoneEngine adds it, since the label lives in the registry, not the plugin)."""

    @classmethod
    @abstractmethod
    def zone_type_from_dict(cls, d: dict) -> ZoneType:
        """Deserialize a plain dict into a ZoneType."""

    # ---- operations on the resolved zone: instance methods -------------------

    @abstractmethod
    def to_dict(self) -> dict:
        """Serialize this zone's resolved bounds to a plain dict."""

    @abstractmethod
    def spatial_mask(self, xyz_map: np.ndarray) -> np.ndarray:
        """Boolean mask: which map-frame points fall inside the zone's 3D extent."""

    @abstractmethod
    def projective_mask(
        self,
        az: np.ndarray,
        el: np.ndarray,
        lidar_position: np.ndarray,
        y_padding: dict[str, float],
        z_padding: dict[str, float],
    ) -> np.ndarray:
        """Boolean mask: which point bearings fall in the zone's angular cone."""

    @abstractmethod
    def expected_fields(self, lidar_pos: np.ndarray) -> dict:
        """Lidar-relative field dict for the viz ExpectedZone msg."""

    # ---- visualization: ROS marker msgs live INSIDE the method --------------

    @abstractmethod
    def build_markers(self, array, stamp, marker_id: int) -> int:
        """Append this zone's RViz markers to `array`, returning the next id.

        Import visualization_msgs / geometry_msgs (and the shared
        zone_plugins.marker_helpers) INSIDE this method so pure consumers like
        the filter never load the visualization message types.
        """

    # ---- generation-time: heavy imports live INSIDE the method ---------------

    @classmethod
    @abstractmethod
    def construct_urdf_link(cls, props: dict):
        """Build this zone's URDF link from environment-config props.

        Import Box/Cylinder/etc. from urdf_parser_py INSIDE this method so pure
        consumers never load it. Returns a urdf_parser_py Link.
        """

    @classmethod
    @abstractmethod
    def roi_fields(cls, props: dict) -> dict:
        """Build this geometry's ROI fields (for roi.yaml) from environment-config props.

        The generation-time counterpart to construct_urdf_link: a pure dict transform
        emitting the per-geometry keys the ROILoader expects (planar → width/z_bounds,
        cylindrical → radius/height + paddings)."""

    @classmethod
    def lateral_half_extent(cls, props: dict) -> float:
        """Half the zone's lateral (Y) extent, from environment-config props — used for
        pan-sweep coverage and edge-to-edge zone placement.

        Optional: the default treats a zone as a point (0.0); geometries with lateral
        size override it (planar → length/2, cylindrical → radius)."""
        return 0.0
