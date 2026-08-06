import os
import yaml


class LidarMetricsRegistry():

    # registry.yaml key suffix that marks a per-geometry metric list.
    _ZONE_KEY_SUFFIX = '_zone_metrics'

    def __init__(self):
        # Nested: { geometry_token: { metric_name: info } }
        self.enabled_metrics_by_geometry = {}
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.registry_yaml = os.path.join(current_dir, 'registry.yaml')

    def register_metrics(self) -> dict:
        """Parse registry.yaml into { geometry_token: { metric_name: info } }.

        Each top-level key under `lidar_metrics` named "<geo>_zone_metrics"
        contributes a geometry bucket "<geo>" (e.g. planar_zone_metrics ->
        'planar'). Adding a new geometry is just a new key here — no code change.
        """
        with open(self.registry_yaml, 'r') as f:
            registry_data = yaml.safe_load(f)

        for key, metrics in (registry_data.get('lidar_metrics') or {}).items():
            if not key.endswith(self._ZONE_KEY_SUFFIX):
                continue
            geometry = key[: -len(self._ZONE_KEY_SUFFIX)]
            bucket = self.enabled_metrics_by_geometry.setdefault(geometry, {})

            for metric in metrics or []:
                if not metric.get('enabled'):
                    continue
                bucket[metric['name']] = {
                    'description': metric['description'],
                    'executable': metric['executable'],
                    'category': metric['category'],
                    'return_type': metric['return_type'],
                    'geometry': geometry,
                }

        return self.enabled_metrics_by_geometry

    def shutdown(self) -> None:
        # Clear the registry and any related resources if needed
        self.enabled_metrics_by_geometry.clear()
