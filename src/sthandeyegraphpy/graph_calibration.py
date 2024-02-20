from __future__ import annotations

from typing import List, Tuple, Union

import numpy as np

from ._core import spatial_calibration_graph, spatial_calibration_visp


class HandEyeCalibration:
    def __init__(
        self, camera_params: Union[list, tuple, np.ndarray], pattern2d: np.ndarray # noqa: UP007
    ) -> None:
        if not isinstance(camera_params, (list, tuple, np.ndarray)):
            msg = f"camera_params needs to be of type list|tuple|np.ndarray, provided {type(camera_params)}"
            raise ValueError(msg)

        if isinstance(camera_params, np.ndarray):
            if len(camera_params.shape) != 1:
                msg = "Only 1D arrays accepeted!"
                raise ValueError(msg)

            if camera_params.shape[-1] != 6:
                msg = "Expected 6 parameters: [sx, sy, stheta, fcam, cx, cy]"
                raise ValueError(msg)

        if isinstance(camera_params, (list, tuple)) and len(camera_params) != 6:
            msg = "Expected 6 parameters: [sx, sy, stheta, fcam, cx, cy]"
            raise ValueError(msg)

        if len(pattern2d.shape) != 2:
            msg = "2d pattern needs to be of shape 2xn!"
            raise ValueError(msg)

        self._camera: Union[list, tuple, np.ndarray] = camera_params # noqa: UP007
        self._pattern: np.ndarray = pattern2d
        self._poses: List[np.ndarray] = [] # noqa: UP006
        self._patterns: List[np.ndarray] = [] # noqa: UP006
        self._pattern_pose: np.ndarray = None
        self._hand2eye: np.ndarray = None

    def add_sample(self, projected_pattern: np.ndarray, ee_pose: np.ndarray) -> None:
        if len(projected_pattern.shape) != 2:
            msg = "2d pattern needs to be of shape 2xn!"
            raise ValueError(msg)

        if len(ee_pose.shape) != 1 or len(ee_pose.shape[0]) != 6:
            msg = "Expceted 6d pose vector!"
            raise ValueError(msg)

        self._poses.append(ee_pose)
        self._patterns.append(projected_pattern)

    def optimize(self) -> Tuple[np.ndarray, np.ndarray, bool]: # noqa: UP006

        camera_matrix = self.camera_matrix
        pattern3d = np.zeros((3, self._pattern.shape[-1]))
        pattern3d[:-1, :] = self._pattern

        if self._hand2eye is None:
            self._hand2eye = np.zeros((6,))
            self._pattern_pose = np.zeros_like(self._hand2eye)

            self._hand2eye, self._pattern_pose, _ = spatial_calibration_visp(
                camera_matrix,
                pattern3d,
                self._patterns,
                self._poses,
                self._hand2eye,
                self._pattern_pose,
            )
        self._hand2eye, self._pattern_pose, ok = spatial_calibration_graph(
            camera_matrix,
            pattern3d,
            self._patterns,
            self._poses,
            self._hand2eye,
            self._pattern_pose,
        )

        return self._hand2eye, self._pattern_pose, ok

    @property
    def camera_matrix(self) -> np.ndarray:
        out = np.eye(3)
        out[0, 0] = self._camera[0] * self._camera[3]
        out[0, 1] = self._camera[2]
        out[1, 1] = self._camera[1] * self._camera[3]
        out[0, 2] = self._camera[4]
        out[1, 2] = self._camera[5]
        return out

    @property
    def hand2eye(self) -> np.ndarray:
        return self._hand2eye

    @hand2eye.setter
    def hand2eye(self, other: np.ndarray) ->  None:
        if len(other.shape) != 1 or other.shape[0] != 6:
            msg = "Expected 6D pose vector!"
            raise ValueError(msg)
        self._hand2eye = other

    @property
    def pattern_pose(self) -> np.ndarray:
        return self._pattern_pose

    @pattern_pose.setter
    def pattern_pose(self, other: np.ndarray) ->  None:
        if len(other.shape) != 1 or other.shape[0] != 6:
            msg = "Expected 6D pose vector!"
            raise ValueError(msg)
        self._pattern_pose = other