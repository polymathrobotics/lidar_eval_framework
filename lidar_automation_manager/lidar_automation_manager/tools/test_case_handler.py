# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

from rclpy.parameter import Parameter

from lidar_automation_manager.tools.utils import (
    AngleTestCase,
    ParameterBooleanTestCase,
    ParameterNumericalTestCase,
)


# Mapping from YAML 'type' string -> (scalar Parameter.Type, array Parameter.Type, ParameterValue field name).
_TYPE_MAP = {
    'bool':   (Parameter.Type.BOOL,    Parameter.Type.BOOL_ARRAY,    'bool_array_value'),
    'int':    (Parameter.Type.INTEGER, Parameter.Type.INTEGER_ARRAY, 'integer_array_value'),
    'double': (Parameter.Type.DOUBLE,  Parameter.Type.DOUBLE_ARRAY,  'double_array_value'),
    'string': (Parameter.Type.STRING,  Parameter.Type.STRING_ARRAY,  'string_array_value'),
}

class TestCaseHandler:

    def __init__(self, node):
        self._node = node
        self.test_cases = []
        self._cursor = 0

    def load_test_cases(self, angles, parameter_names):
        cases = []

        for angle in angles:
            cases.append(AngleTestCase(angle=angle))

        for name in parameter_names:
            self._node.declare_parameter(f'{name}.type', Parameter.Type.STRING)
            type_str = self._node.get_parameter(f'{name}.type').get_parameter_value().string_value

            if type_str not in _TYPE_MAP:
                raise ValueError(
                    f"parameter '{name}' has unsupported type '{type_str}'; "
                    f"expected one of {sorted(_TYPE_MAP)}"
                )
            scalar_type, param_type, value_field = _TYPE_MAP[type_str]

            self._node.declare_parameter(f'{name}.values', param_type)
            values = list(getattr(self._node.get_parameter(f'{name}.values').get_parameter_value(), value_field))

            self._node.declare_parameter(f'{name}.path', Parameter.Type.STRING)
            path = self._node.get_parameter(f'{name}.path').get_parameter_value().string_value

            self._node.declare_parameter(f'{name}.default', scalar_type)
            default_value = self._node.get_parameter(f'{name}.default').value

            for value in values:
                if value == default_value:
                    self._node.get_logger().info(
                        f"Skipping {name}={value!r} — matches default, already covered by base"
                    )
                    continue
                if 'bool' == type_str:
                    cases.append(ParameterBooleanTestCase(parameter=name, value=value, path=path))
                elif 'int' == type_str:
                    # Preserve int — set_driver_param writes the value back into the driver's
                    # YAML config, and some drivers (Hesai yaml-cpp) refuse to parse a double
                    # like 30.0 into an int-typed field, throwing TypedBadConversion.
                    cases.append(ParameterNumericalTestCase(parameter=name, value=int(value), path=path))
                elif 'double' == type_str:
                    cases.append(ParameterNumericalTestCase(parameter=name, value=float(value), path=path))
                else:
                    raise NotImplementedError(
                        f"no TestCase struct defined for parameter type '{type_str}'"
                    )

        self.test_cases = cases
        self._cursor = 0

    def next_test_case(self):

        if self._cursor >= len(self.test_cases):
            return None
        case = self.test_cases[self._cursor]
        self._cursor += 1
        return case

    def reset(self):
        self._cursor = 0
