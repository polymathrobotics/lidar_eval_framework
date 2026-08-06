# Copyright (c) 2025-present Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import numpy as np


def quat_to_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:

    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)    ],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)    ],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


def build_transform_matrix(translation, quaternion) -> np.ndarray:

    rotation = quat_to_rotation_matrix(quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    mat = np.eye(4)
    mat[:3, :3] = rotation
    mat[:3, 3] = [translation.x, translation.y, translation.z]
    return mat


def apply_transform(xyz: np.ndarray, transform_4x4: np.ndarray) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to an (N, 3) point array.

    Args:
        xyz: Shape (N, 3) array of points in the source frame.
        transform_4x4: Shape (4, 4) homogeneous transform matrix.

    Returns:
        Shape (N, 3) array of points in the target frame.
    """
    ones = np.ones((xyz.shape[0], 1))
    return (transform_4x4 @ np.hstack([xyz, ones]).T).T[:, :3]
