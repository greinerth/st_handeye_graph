#include <variant>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <st_handeye/st_handeye.hpp>

namespace py = pybind11;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

Eigen::Isometry3d pose2isometry3d(const Vector6d &pose){

  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  Eigen::Vector3d ortientation = pose.block(0, 0, 3, 1);
  Eigen::Vector3d position = pose.block(3, 0, 3, 1);

  double angle = ortientation.norm();

  // identity transform if the angle is 0.0
  if (angle != 0){
    out.translation() = position;
    out.linear() = Eigen::AngleAxisd(angle, ortientation / angle).toRotationMatrix();
  }
  return out;
}

Vector6d isometry2pose(const Eigen::Isometry3d& iso){
    Vector6d out;
    Eigen::AngleAxisd axis(iso.linear());
    out.block(0, 0, 3, 1) = axis.axis() * axis.angle();
    out.block(3, 0, 3, 1) = iso.translation();
    return out;
}

PYBIND11_MODULE(_core, m) {

  py::class_<st_handeye::OptimizationParams>(m, "OptimizationParams")
  .def(py::init<>(), R"pdoc(
    """ Initialize the optimization parameters."""
  )pdoc")
  .def_readwrite("world2hand_inf_scale_trans", &st_handeye::OptimizationParams::world2hand_inf_scale_trans)
  .def_readwrite("world2hand_inf_scale_rot", &st_handeye::OptimizationParams::world2hand_inf_scale_rot)
  .def_readwrite("pattern2d_inf_scale", &st_handeye::OptimizationParams::world2hand_inf_scale_rot)
  .def_readwrite("source_inf_scale", &st_handeye::OptimizationParams::source_inf_scale)
  .def_readwrite("robust_kernel_handpose_delta", &st_handeye::OptimizationParams::robust_kernel_handpose_delta)
  .def_readwrite("robust_kernel_projection_delta", &st_handeye::OptimizationParams::robust_kernel_projection_delta)
  .def_readwrite("robust_kernel_source_delta", &st_handeye::OptimizationParams::robust_kernel_source_delta)
  .def_readwrite("num_iterations", &st_handeye::OptimizationParams::num_iterations)
  .def_readwrite("robust_kernel_handpose", &st_handeye::OptimizationParams::robust_kernel_handpose)
  .def_readwrite("robust_kernel_projection", &st_handeye::OptimizationParams::robust_kernel_projection)
  .def_readwrite("robust_kernel_source", &st_handeye::OptimizationParams::robust_kernel_source)
  .def_readwrite("solver_name", &st_handeye::OptimizationParams::solver_name);

  m.def("spatial_calibration_graph", [](
                                        Eigen::Ref<const Eigen::Matrix3d> camera_matrix,
                                        Eigen::Ref<const Eigen::MatrixXd> pattern_3d,
                                        const std::vector<Eigen::MatrixXd> &pattern_2ds,
                                        const std::vector<Eigen::Ref<const Vector6d>> &world2hands,
                                        Eigen::Ref<const Vector6d> hand2eye,
                                        Eigen::Ref<const Vector6d> object2world,
                                        const st_handeye::OptimizationParams &params) -> py::tuple {
                                          std::vector<Eigen::Isometry3d> _world2hands(world2hands.size());

                                          for (auto i = 0; i < world2hands.size(); i++)
                                            _world2hands[i] = pose2isometry3d(world2hands[i]);

                                          Eigen::Isometry3d _hand2eye = pose2isometry3d(hand2eye);
                                          Eigen::Isometry3d _object2world = pose2isometry3d(object2world);
                                          bool ok = st_handeye::spatial_calibration_graph(camera_matrix, pattern_3d, _world2hands, pattern_2ds, _hand2eye, _object2world, params);
                                          py::tuple out = py::make_tuple(isometry2pose(_hand2eye), isometry2pose(_object2world), ok);
                                          return out;
                                        },
                                        py::arg("camera_matrix"),
                                        py::arg("pattern_3d"),
                                        py::arg("pattern_2ds"),
                                        py::arg("world2hands"),
                                        py::arg("hand2eye"),
                                        py::arg("object2world"),
                                        py::arg("params") = st_handeye::OptimizationParams()),
                                        R"pdoc(
                                          Perform the graph optimization.

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
                                        )pdoc";

  m.def("spatial_calibration_visp", [](
                                        Eigen::Ref<const Eigen::Matrix3d> camera_matrix,
                                        Eigen::Ref<const Eigen::MatrixXd> pattern_3d,
                                        const std::vector<Eigen::MatrixXd> &pattern_2ds,
                                        const std::vector<Eigen::Ref<const Vector6d>> &world2hands,
                                        Eigen::Ref<const Vector6d> hand2eye,
                                        Eigen::Ref<const Vector6d> object2world,
                                        const st_handeye::OptimizationParams &params) -> py::tuple {
                                          std::vector<Eigen::Isometry3d> _world2hands(world2hands.size());

                                          for (auto i = 0; i < world2hands.size(); i++)
                                            _world2hands[i] = pose2isometry3d(world2hands[i]);

                                          Eigen::Isometry3d _hand2eye = pose2isometry3d(hand2eye);
                                          Eigen::Isometry3d _object2world = pose2isometry3d(object2world);
                                          bool ok = st_handeye::spatial_calibration_visp(camera_matrix, pattern_3d, _world2hands, pattern_2ds, _hand2eye, _object2world, params);
                                          py::tuple out = py::make_tuple(isometry2pose(_hand2eye), isometry2pose(_object2world), ok);
                                          return out;
                                        },
                                        py::arg("camera_matrix"),
                                        py::arg("pattern_3d"),
                                        py::arg("pattern_2ds"),
                                        py::arg("world2hands"),
                                        py::arg("hand2eye"),
                                        py::arg("object2world"),
                                        py::arg("params") = st_handeye::OptimizationParams(),
                                        R"pdoc(
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
                                        )pdoc"
                                        );
}

