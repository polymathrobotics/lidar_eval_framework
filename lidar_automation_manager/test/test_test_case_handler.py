# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""High-level tests for TestCaseHandler — turning declared sweeps into test cases."""

import pytest
from rclpy.parameter import Parameter

# Aliased: pytest would otherwise try to collect the Test*-prefixed class.
from lidar_automation_manager.tools.test_case_handler import TestCaseHandler as CaseHandler
from lidar_automation_manager.tools.utils import (
    AngleTestCase,
    ParameterBooleanTestCase,
    ParameterNumericalTestCase,
)


class StubNode:
    """Serves parameters from a dict, standing in for an rclpy Node."""

    def __init__(self, params):
        self._params = params
        self.messages = []

    def declare_parameter(self, name, parameter_type):
        if name not in self._params:
            raise KeyError(f'undeclared test parameter: {name}')

    def get_parameter(self, name):
        return Parameter(name, value=self._params[name])

    def get_logger(self):
        return self

    def info(self, message):
        self.messages.append(message)


def test_angles_become_one_case_each():
    handler = CaseHandler(StubNode({}))

    handler.load_test_cases([0, 15, 30], [])

    assert [type(c) for c in handler.test_cases] == [AngleTestCase] * 3
    assert [c.angle for c in handler.test_cases] == [0, 15, 30]
    assert all(c.test_type == 'angle' for c in handler.test_cases)


def test_numeric_sweep_skips_the_value_already_covered_by_base():
    node = StubNode({
        'rpm.type': 'int',
        'rpm.values': [600, 1200],
        'rpm.path': '/cfg/hesai.yaml',
        'rpm.default': 600,
    })
    handler = CaseHandler(node)

    handler.load_test_cases([], ['rpm'])

    assert len(handler.test_cases) == 1
    case = handler.test_cases[0]
    assert isinstance(case, ParameterNumericalTestCase)
    assert case.parameter == 'rpm' and case.value == 1200 and case.path == '/cfg/hesai.yaml'
    # Ints stay ints — some drivers refuse to parse 1200.0 into an int field.
    assert isinstance(case.value, int)
    assert any('matches default' in m for m in node.messages)


def test_double_and_bool_sweeps_build_their_own_case_types():
    node = StubNode({
        'cut_angle.type': 'double', 'cut_angle.values': [0.5, 1.5],
        'cut_angle.path': '/cfg/a.yaml', 'cut_angle.default': 0.0,
        'dual_return.type': 'bool', 'dual_return.values': [True, False],
        'dual_return.path': '/cfg/b.yaml', 'dual_return.default': False,
    })
    handler = CaseHandler(node)

    handler.load_test_cases([], ['cut_angle', 'dual_return'])

    doubles = [c for c in handler.test_cases if isinstance(c, ParameterNumericalTestCase)]
    bools = [c for c in handler.test_cases if isinstance(c, ParameterBooleanTestCase)]
    assert [c.value for c in doubles] == [pytest.approx(0.5), pytest.approx(1.5)]
    assert all(isinstance(c.value, float) for c in doubles)
    assert [c.value for c in bools] == [True]        # False is the default, skipped
    assert bools[0].test_type == 'parameter_bool'


def test_unsupported_parameter_type_is_rejected():
    node = StubNode({'weird.type': 'complex'})
    handler = CaseHandler(node)

    with pytest.raises(ValueError, match='unsupported type'):
        handler.load_test_cases([], ['weird'])


def test_cursor_walks_the_cases_once_then_resets():
    handler = CaseHandler(StubNode({}))
    handler.load_test_cases([0, 15], [])

    assert [handler.next_test_case().angle for _ in range(2)] == [0, 15]
    assert handler.next_test_case() is None

    handler.reset()
    assert handler.next_test_case().angle == 0


def test_loading_again_replaces_the_previous_cases():
    handler = CaseHandler(StubNode({}))
    handler.load_test_cases([0, 15], [])
    handler.next_test_case()

    handler.load_test_cases([90], [])

    assert len(handler.test_cases) == 1
    assert handler.next_test_case().angle == 90
