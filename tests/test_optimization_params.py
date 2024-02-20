import sthandeyegraphpy._core as m


def test_optimization_params():
    obj = m.OptimizationParams()
    assert obj.world2hand_inf_scale_trans == 0.01
    assert obj.world2hand_inf_scale_rot == 1.0
    assert obj.pattern2d_inf_scale == 1.0
    assert obj.source_inf_scale == 1.0
    assert obj.robust_kernel_handpose_delta == 0.01
    assert obj.robust_kernel_projection_delta == 1.0
    assert obj.robust_kernel_source_delta == 1.0
    assert obj.num_iterations == 8192
    assert obj.robust_kernel_handpose == "Huber"
    assert obj.robust_kernel_projection == "Huber"
    assert obj.robust_kernel_source == "Huber"
    assert obj.solver_name == "lm_var_cholmod"