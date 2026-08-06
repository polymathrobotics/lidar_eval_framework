from dataclasses import replace
from lidar_metrics.reporter import LidarMetricsReporter
from lidar_metrics.registry import LidarMetricsRegistry
from lidar_metrics.metric_interfaces.metrics_base import MetricsBase
from lidar_metrics.metric_params_overrides.override_interface.base import OverrideInterfaceBase

import numpy as np
import os
import importlib.util
import inspect
import yaml



class LidarMetricsEngine():


    def __init__(self, output_folder_path : str, horizontal_resolution: float, vertical_resolution: float) -> None:

        self.plugin_registry = {}
        self.plugin_instances = {}
        self.metrics_reporter = LidarMetricsReporter(output_folder_path)
        self.lidar_registry = LidarMetricsRegistry()
        self.final_results = {}
        self.current_report_stem = None
        self.horizontal_resolution = horizontal_resolution
        self.vertical_resolution = vertical_resolution
        self.output_folder_path = output_folder_path
        self._profiles = None
        # Base config (the documented, git-tracked source) and the config the plugins
        # actually load. They're the same until override_metrics_params writes a
        # derived copy with the overridden params, leaving the base file untouched.
        self._base_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
        self._runtime_config_path = self._base_config_path
        self.folder_paths = {
            "params_override_dir" : "lidar_metrics.metric_params_overrides"
        }

    def start_new_test_run(self, folder_path: str, report_stem: str) -> None:
        self.current_report_stem = report_stem
        self.metrics_reporter.generate_new_test_run_folder(folder_path)


    def set_base(self, profiles) -> None:
        self._profiles = profiles
        self.override_metrics_params()


    def override_metrics_params(self):

        # function design
        # 1. go to directoy specificed in the self.folder_paths
        # 2. executable schema is {metric_name}__{param_name}__override
        # 3. read each executable and parse both metric name and param_override
        # 4. create executable and call retrieve param
        # 5. change the appropriate param in configs

        # 1. Resolve the params-override directory. The value in self.folder_paths is
        # a dotted package path (same convention as the metric registry), so turn it
        # into a filesystem directory to iterate.
        override_package = self.folder_paths["params_override_dir"]
        spec = importlib.util.find_spec(override_package)
        if spec is None or not spec.submodule_search_locations:
            return
        params_override_dir = spec.submodule_search_locations[0]

        # Start from the base config; overrides are applied to this copy only.
        with open(self._base_config_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
        params = config.setdefault("lidar_metrics_parameters", {})

        changed = False
        for filename in os.listdir(params_override_dir):
            if not filename.endswith(".py") or filename.startswith("__"):
                continue
            module_stem = filename[: -len(".py")]
            parts = module_stem.split("__")
            if len(parts) != 3 or parts[2] != "override":
                continue
            metric_name, param_name, _ = parts

            # 4. Import the override module, find its OverrideInterfaceBase subclass
            # (named anything, e.g. SpatialDropoutOverride), instantiate it with the
            # profiles, and retrieve the value.
            override_module = importlib.import_module(f"{override_package}.{module_stem}")
            override_cls = next(
                (cls for _, cls in inspect.getmembers(override_module, inspect.isclass)
                 if issubclass(cls, OverrideInterfaceBase) and cls is not OverrideInterfaceBase),
                None,
            )
            if override_cls is None:
                continue
            value = override_cls(self._profiles).retrieve_param()

            # 5. Apply the retrieved value to this metric's params in the config copy.
            params.setdefault(metric_name, {})[param_name] = value
            changed = True

        # Write the overridden params back into config.yaml itself. NOTE: this
        # rewrites the file via yaml.safe_dump, which strips comments — switch to
        # ruamel.yaml round-trip if comments must be preserved.
        if changed:
            with open(self._base_config_path, "w", encoding="utf-8") as f:
                yaml.safe_dump(config, f, sort_keys=False)


    def load_registry(self) -> None:

        self.plugin_registry = self.lidar_registry.register_metrics()

    def run(
        self,
        spatial_by_zone: dict[str, np.ndarray],
        projective_by_zone: dict[str, np.ndarray],
    ) -> None:

        # Each scan: only feed update(). compute()/shutdown() are deferred to
        # report() so metrics can aggregate over the full run before reducing.
        if self._profiles is None:
            return  # no base profiles wired in yet — nothing to route

        # Each pointcloud is keyed by zone name; look that name up in the stored
        # profiles to learn the zone's geometry (carried by its bounds type).
        zone_geometry = {
            zb.name: type(zb).__name__.removesuffix('ZoneBounds').lower()
            for zb in self._profiles.zone_bounds
        }

        # plugin_registry is { geometry: { metric_name: info } }. Apply each
        # geometry's metric set to only that geometry's zones.
        for geometry, metrics in self.plugin_registry.items():
            spatial_sub = {z: p for z, p in spatial_by_zone.items() if zone_geometry.get(z) == geometry}
            projective_sub = {z: p for z, p in projective_by_zone.items() if zone_geometry.get(z) == geometry}
            if not spatial_sub and not projective_sub:
                continue

            # Profiles restricted to this geometry so metrics that loop
            # profiles.zone_bounds never see another geometry's zones.
            geom_profiles = replace(
                self._profiles,
                zone_bounds=[zb for zb in self._profiles.zone_bounds
                             if zone_geometry.get(zb.name) == geometry],
            )

            for metric_name, metric_info in metrics.items():
                key = (geometry, metric_name)
                category = metric_info['category']
                if key not in self.plugin_instances:
                    self.plugin_instances[key] = self.create_plugins(
                        metric_info['executable'], category, metric_name, geometry,
                        spatial_sub, projective_sub, geom_profiles,
                    )
                zone_dict = spatial_sub if category == 'spatial' else projective_sub
                self.plugin_instances[key].update(zone_dict)


    def report(self) -> None:

        # Zone names for this run, longest first so prefix matching handles
        # names containing underscores (e.g. "black_half_cylinder_<sub>").
        zone_names = sorted(
            (zb.name for zb in self._profiles.zone_bounds),
            key=len, reverse=True,
        ) if self._profiles is not None else []

        # End-of-run reduction, pivoted to { zone_name: { metric_name: {...} } }.
        # Each metric returns zone-prefixed keys ("<zone>_<sub>") or a value
        # keyed directly by zone name; those are attributed to the zone. Keys
        # that match no zone (run-global scalars) land under "__global__".
        for (_geometry, metric_name), plugin in self.plugin_instances.items():
            result = plugin.compute()
            plugin.shutdown()

            if not isinstance(result, dict):
                self.final_results.setdefault('__global__', {})[metric_name] = result
                continue

            for key, value in result.items():
                zone = self._match_zone(key, zone_names)
                if zone is None:
                    self.final_results.setdefault('__global__', {}).setdefault(metric_name, {})[key] = value
                elif key == zone:
                    # value already scoped to the zone (keyed by zone name)
                    self.final_results.setdefault(zone, {})[metric_name] = value
                else:
                    sub = key[len(zone) + 1:]  # strip "<zone>_"
                    self.final_results.setdefault(zone, {}).setdefault(metric_name, {})[sub] = value

        # Carry the lidar's map-frame position into the report so downstream consumers
        # (e.g. the reporting node's viz transform) can shift coordinates into the lidar
        # frame offline — no live TF lookup needed.
        if self._profiles is not None and getattr(self._profiles, 'lidar_position', None) is not None:
            self.final_results.setdefault('__global__', {})['lidar_position'] = [
                float(c) for c in self._profiles.lidar_position
            ]

        self.metrics_reporter.receive_report(self.final_results)
        self.metrics_reporter.generate_new_report(self.current_report_stem)
        self.metrics_reporter.report()
        self.metrics_reporter.reset()
        self.reset()


    def create_plugins(
        self,
        executable_name: str,
        category: str,
        metric_name: str,
        geometry: str,
        spatial_by_zone: dict[str, np.ndarray],
        projective_by_zone: dict[str, np.ndarray],
        profiles=None,
    ) -> object:

        module = __import__(
            f"lidar_metrics.zone_metrics.{geometry}_zones.{category}_metrics.{executable_name}",
            fromlist=[executable_name],
        )
        plugin_class = getattr(module, metric_name)

        if category == 'spatial':
            plugin_instance = plugin_class(spatial_by_zone, profiles)
        elif category == 'projective':
            plugin_instance = plugin_class(projective_by_zone, profiles)
        else:
            raise ValueError(f"Unknown metric category {category!r} for plugin {executable_name}")

        if not isinstance(plugin_instance, MetricsBase):
            raise TypeError(f"Plugin {executable_name} does not inherit from MetricsBase")

        # Inject sensor context before setup() so metrics that consume the
        # angular resolutions in setup() (e.g. PointDensityYieldPerZone,
        # PointDensityHeatMapPerZone) see the real values rather than the
        # base-class defaults of 0.0.
        plugin_instance.horizontal_resolution = self.horizontal_resolution
        plugin_instance.vertical_resolution = self.vertical_resolution

        plugin_instance.initialize_lib(self._runtime_config_path)
        plugin_instance.setup()
        return plugin_instance


    @staticmethod
    def _match_zone(key: str, zone_names_longest_first: list[str]) -> str | None:
        """Return the zone a result key belongs to, or None for run-global keys.

        Matches an exact zone name (value keyed directly by zone) or a
        "<zone>_..." prefix. Caller must pass zone names longest-first so a name
        that is a prefix of another doesn't shadow the longer match.
        """
        for zone in zone_names_longest_first:
            if key == zone or key.startswith(zone + '_'):
                return zone
        return None




    def reset(self) -> None:

        self.final_results.clear()

    def shutdown(self):

        # handles any cleanup needed for the engine and the plugins
        pass
