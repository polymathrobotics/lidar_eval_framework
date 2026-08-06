# a script which retrieves the metrics and reports them in a nice format in yaml somehow

import yaml
import numpy as np
from pathlib import Path
from typing import Any, Dict, Optional
import os
class LidarMetricsReporter:


    def __init__(self, output_path: str = 'metrics_results'):

        self.reporting_metrics: Dict[str, Any] = {}
        self.current_output_folder: Optional[Path] = None
        self.current_output_file: Optional[Path] = None
        results_path = Path(output_path)
        results_path.mkdir(parents=True, exist_ok=True)
        self.output_path = results_path

    def generate_new_test_run_folder(self, output_folder_name: str) -> None:

        # 1. Clean the input: remove leading slashes to prevent 'absolute path' hijacking
        clean_name = output_folder_name.lstrip("/")

        # 2. Use the / operator for joining Path objects
        target_path = self.output_path / clean_name

        # 3. Create the directory
        target_path.mkdir(parents=True, exist_ok=True)

        # 4. Store it
        self.current_output_folder = target_path


    def generate_new_report(self, output_file_stem: str) -> None:
        if self.current_output_folder is None:
            raise RuntimeError("Call generate_new_test_run_folder() before generate_new_report().")

        name = output_file_stem.strip()

        _KNOWN_EXTENSIONS = {'.yaml', '.yml', '.txt', '.json', '.csv'}
        if Path(name).suffix.lower() in _KNOWN_EXTENSIONS:
            raise ValueError(f"Pass only the file name without extension (got: {output_file_stem!r}).")

        self.current_output_file = self.current_output_folder / f"{name}.yaml"

    def receive_report(self, report_data: dict):
        # Engine now hands us the final per-metric result directly — each metric
        # has already done its own end-of-run reduction in compute().
        self.reporting_metrics.update(report_data)



    def report(self):
        if self.current_output_file is None:
            raise RuntimeError("Call generate_new_report() before report().")

        payload = self._to_yaml_friendly(self.reporting_metrics)

        self.current_output_file.parent.mkdir(parents=True, exist_ok=True)
        with open(self.current_output_file, "w") as f:
            yaml.safe_dump(payload, f, sort_keys=True, default_flow_style=False)



    @staticmethod
    def _to_yaml_friendly(obj: Any) -> Any:
        """Convert numpy types / arrays and other non-serializables into YAML-safe Python types."""
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        if isinstance(obj, (np.floating,)):
            return float(obj)
        if isinstance(obj, (np.integer,)):
            return int(obj)
        if isinstance(obj, dict):
            return {str(k): LidarMetricsReporter._to_yaml_friendly(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [LidarMetricsReporter._to_yaml_friendly(v) for v in obj]
        return obj

    def reset(self) -> None:

        self.reporting_metrics.clear()

    def shutdown(self) -> None:
        self.reset()
        self.current_output_folder = None
        self.current_output_file = None
