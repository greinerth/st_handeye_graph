from __future__ import annotations

import numpy as np

__all__ = [
    "OptimizationParams",
    "spatial_calibration_graph",
    "spatial_calibration_visp",
]

class OptimizationParams:
    num_iterations: int
    pattern2d_inf_scale: float
    robust_kernel_handpose: str
    robust_kernel_handpose_delta: float
    robust_kernel_projection: str
    robust_kernel_projection_delta: float
    robust_kernel_source: str
    robust_kernel_source_delta: float
    solver_name: str
    source_inf_scale: float
    world2hand_inf_scale_rot: float
    world2hand_inf_scale_trans: float
    def __init__(self) -> None:
        """
        \"\"\" Initialize the optimization parameters.\"\"\"
        """

def spatial_calibration_graph(
    camera_matrix: np.ndarray[np.float64[3, 3], np.ndarray.flags.f_contiguous],
    pattern_3d: np.ndarray[np.float64[m, n], np.ndarray.flags.f_contiguous],
    pattern_2ds: list[np.ndarray[np.float64[m, n]]],
    world2hands: list[np.ndarray[np.float64[6, 1]]],
    hand2eye: np.ndarray[np.float64[6, 1]],
    object2world: np.ndarray[np.float64[6, 1]],
    params: OptimizationParams = ...,
) -> tuple: ...
def spatial_calibration_visp(
    camera_matrix: np.ndarray[np.float64[3, 3], np.ndarray.flags.f_contiguous],
    pattern_3d: np.ndarray[np.float64[m, n], np.ndarray.flags.f_contiguous],
    pattern_2ds: list[np.ndarray[np.float64[m, n]]],
    world2hands: list[np.ndarray[np.float64[6, 1]]],
    hand2eye: np.ndarray[np.float64[6, 1]],
    object2world: np.ndarray[np.float64[6, 1]],
) -> tuple:
    r"""
    Perform the visp optimization.
    :param camera_matrix: The intrinsic camera matrix :math:`\boldsymbol{K} \in \mathbb{R}^{3 \times 4}`.
    :type camera_matrix: np.ndarray
    :param pattern_3d: 3D calibration pattern.
    :type pattern_3d: np.ndarray
    :param pattern_2d: Projected pattern :math:`\boldsymbol{P}_{2d} \in \mathbb{R}^{2 \times n_r n_c}`,
        where :math:`n_r, n_c` denote the number of rows and the number of colums of the pattern respectively.
    :type pattern_2d: np.ndarray
    :param world2hands: List of 6D hand poses.
    :type world2hands: List[np.ndarray].
    :param hand2eye: Initial relative pose from EE to camera.
    :type hand2eye: np.ndarray
    :param object2world: Inital 6D pose estimate of the calibration pattern.
    :type object2world: np.ndarray
    :param params: Optmization parameters.
    :type params, optional: OptimizationParams, defaults to OptimizationParams().
    :returns: Estimated relative 6D pose to EE, pose of the calibration pattern, status of the optimization.
    :rtype: Tuple[np.ndarray, np.ndarray, bool]
    """
