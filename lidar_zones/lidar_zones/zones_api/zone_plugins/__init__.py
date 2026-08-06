# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

"""Per-geometry zone plugins.

One module per geometry, each defining a `ZoneTypePlugin` subclass plus its
`ZoneType`/`ZoneBounds` dataclasses. The plugin classes are registered with the
ZoneEngine in `zone_engine.PLUGIN_CLASSES`; adding a geometry = add a module here
and list its plugin there.
"""
