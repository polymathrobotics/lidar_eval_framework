# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.

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
