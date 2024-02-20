from __future__ import annotations

from itertools import product

import numpy as np
from spatialmath import SE3

from sthandeyegraphpy._core import OptimizationParams, spatial_calibration_graph
from sthandeyegraphpy.util import project2d_perspective


def test_spatial_graph():
    height = 480
    width = 640

    sxf = 10
    f = 1
    sx = sxf / f
    sy = sxf / f
    s_theta = 0
    cx = width // 2
    cy = height // 2
    camera_params = np.array([sx, sy, s_theta, f, cx, cy])

    x_coords, y_coords = np.meshgrid(
        np.linspace(-0.35, 0.35, 7), np.linspace(-0.25, 0.25, 5)
    )
    x_coords = x_coords.reshape(-1)
    y_coords = y_coords.reshape(-1)

    pattern3d = np.zeros((4, x_coords.shape[-1]))
    pattern3d[:2, :] = np.vstack([x_coords, y_coords])
    pattern3d[2, :] = 1.0
    pattern3d[-1, :] = 1.0

    theta_x = np.linspace(-15, 15, 3) / np.pi
    theta_y = np.linspace(-15, 15, 5) / np.pi

    camera_matrix = np.eye(3)
    camera_matrix[0, 0] = sx * f
    camera_matrix[1, 1] = sy * f
    camera_matrix[0, 2] = cx
    camera_matrix[1, 2] = cy

    patterns2d = []
    poses = []

    for tx, ty in product(theta_x, theta_y):
        _transform = SE3.RPY(0, tx, ty).data[0]
        _trans = _transform @ np.array([0, 0, 1, 1])
        _campose = np.zeros((4, 4))
        _campose[:3, :3] = _transform[:3, :3].T
        _campose[:-1, -1] = -_trans[:3]
        _campose[-1, -1] = 1
        _campose = SE3(_campose)
        angle, _axis = _campose.angvec()
        _axis *= angle
        _rot = _transform @ pattern3d
        pose = np.zeros((6,))
        pose[:3] = _axis
        pose[3:] = _campose.t

        projected = project2d_perspective(camera_params, width, height, _rot[:-1, :])[0]
        patterns2d.append(projected[:2, :])
        poses.append(pose)

    hand2eye_estimate = np.random.default_rng().normal(0, 1e-3, 6)
    obj2world_estimate = np.zeros((6,))
    obj2world_estimate[-1] = 1
    obj2world_estimate += np.random.default_rng().normal(0, 1e-3, 6)
    params = OptimizationParams()
    pattern3d[-2, :] = 0

    hand2eye, object2world, ok = spatial_calibration_graph(
        camera_matrix,
        pattern3d,
        patterns2d,
        poses,
        hand2eye_estimate,
        obj2world_estimate,
        params,
    )
    diff = object2world - np.array([0, 0, 0, 0, 0, 1])

    assert ok
    assert np.linalg.norm(hand2eye) <= 5e-3
    assert np.linalg.norm(diff) <= 5e-3
