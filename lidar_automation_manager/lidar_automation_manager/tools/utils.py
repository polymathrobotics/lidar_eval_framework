# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass


@dataclass(kw_only=True)
class TestCase:
    test_type: str = ''

@dataclass(kw_only=True)
class ParameterBooleanTestCase(TestCase):
    test_type: str = 'parameter_bool'
    parameter: str
    value: bool
    path: str = ''

@dataclass(kw_only=True)
class ParameterNumericalTestCase(TestCase):
    test_type: str = 'parameter_numerical'
    parameter: str
    value: float
    path: str = ''


@dataclass(kw_only=True)
class AngleTestCase(TestCase):
    test_type: str = 'angle'
    angle: float
