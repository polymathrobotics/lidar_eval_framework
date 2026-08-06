import numpy as np


def points_in_noise_region(pts: np.ndarray, nr) -> np.ndarray:
    """Return boolean mask for points within a noise region's radius.

    Args:
        pts: Shape (N, 3+) array of points in map frame.
        nr: NoiseRegion instance.

    Returns:
        Boolean mask of shape (N,).
    """
    dists = np.linalg.norm(pts[:, :3] - nr.center, axis=1)
    return dists <= nr.radius
