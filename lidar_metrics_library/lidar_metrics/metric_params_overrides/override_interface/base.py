from abc import ABC, abstractmethod


class OverrideInterfaceBase(ABC):
    """Base class for a param-override executable.

    The engine discovers files named ``{metric_name}__{param_name}__override.py``,
    imports each one, instantiates its ``Override`` class with the run's profiles,
    calls ``retrieve_param()``, and writes the returned value into
    ``config.runtime.yaml`` under ``lidar_metrics_parameters[metric_name][param_name]``.
    """

    def __init__(self, profiles):
        self.profiles = profiles

    @abstractmethod
    def retrieve_param(self):
        """Use information about the zones (self.profiles) to compute the value
        that should be written into the config for this metric's parameter."""
