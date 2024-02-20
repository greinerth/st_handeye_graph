"""Utility module."""
from __future__ import annotations

from typing import Tuple

import numpy as np
from numpy.typing import NDArray


def project2d_perspective(
    cam_params: list | tuple | np.ndarray,
    width: int,
    height: int,
    coords: NDArray,
    far: float = -1,
) -> Tuple[NDArray[np.float64], NDArray[np.bool_]]: # noqa: UP006
    """Perform perspective projection of 3D points.

    Args:
        cam_params (list | tuple | np.ndarray): camera parameters [sx, sy, stheta, fcam, cx, cy]
        width (int): image width
        height (int): image height
        coords (NDArray): coordinates for projection
        far (float, optional): Far clipping distance, only used if far > fcam . Defaults to -1.

    Raises:
        ValueError: If type is not of tuple | list | np.ndarray
        ValueError: If cam_params is of type ndarray but is not 1D
        ValueError: If cam_params is of type ndarray and 1D but does not consist of 6 elelements
        ValueError: If cam_params is of type list | tuple but does not consist of 6 elements
        ValueError: If coords are not of shape [3, n]

    Returns:
        Tuple[NDArray[np.float64], NDArray[np.bool_]]: 2D coordinates, visible points
    """
    if not isinstance(cam_params, (list, tuple, np.ndarray)):
        msg = f"cam_params needs to be of type list|tuple|np.ndarray, provided {type(cam_params)}"
        raise ValueError(msg)

    if isinstance(cam_params, np.ndarray):
        if len(cam_params.shape) != 1:
            msg = "Only 1D arrays accepeted!"
            raise ValueError(msg)

        if cam_params.shape[-1] != 6:
            msg = "Expected 6 parameters: [sx, sy, stheta, fcam, cx, cy]"
            raise ValueError(msg)

    if isinstance(cam_params, (list, tuple)) and len(cam_params) != 6:
        msg = "Expected 6 parameters: [sx, sy, stheta, fcam, cx, cy]"
        raise ValueError(msg)

    if len(coords.shape) != 2 or coords.shape[0] != 3:
        msg = "Expected a 3xn array!"
        raise ValueError(msg)

    __visible = coords[2, :] >= 0

    if far > cam_params[3]:
        c_1 = 2.0 / (far - cam_params[3])
        c_2 = -(cam_params[3] + far) / (far - cam_params[3])
        score = c_1 * coords[2, :] + c_2
        __visible &= score > -1
        __visible &= score < +1

    __out = np.zeros((2, coords.shape[-1]))
    __out[0, :] = (
        coords[0, :] * cam_params[0] + coords[1, :] * cam_params[2]
    ) / coords[-1, :] * cam_params[3] + cam_params[4]
    __out[1, :] = (
        coords[1, :] / coords[-1, :] * cam_params[1] * cam_params[3] + cam_params[5]
    )
    __visible &= __out[0, :] >= 0
    __visible &= __out[0, :] < width
    __visible &= __out[1, :] >= 0
    __visible &= __out[1, :] < height

    return __out, __visible
