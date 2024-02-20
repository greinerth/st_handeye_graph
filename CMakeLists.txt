cmake_minimum_required(VERSION 3.15...3.26)
project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)
find_package(g2o REQUIRED)
find_package(VISP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED filesystem program_options)

add_compile_options(-std=c++17 -O3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

include_directories(
	include
        ${G2O_INCLUDE_DIRS}
        ${VISP_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(${g2o_LIBRARY_DIRS})

###########
## Build ##
###########

add_library(st_handeye_graph
  src/st_handeye_graph.cpp
)
target_link_libraries(st_handeye_graph
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_SLAM3D_ADDONS}
  ${G2O_INCLUDE_DIRS}
)
target_include_directories(st_handeye_graph PRIVATE ${G2O_INCLUDE_DIRS})

# add_library(st_handeye_graph_xray
#   src/st_handeye_graph_xray.cpp
# )
# target_link_libraries(st_handeye_graph_xray
#   ${Boost_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${g2o_TYPES_DATA}
#   ${g2o_CORE_LIBRARY}
#   ${g2o_STUFF_LIBRARY}
#   ${g2o_SOLVER_PCG}
#   ${g2o_SOLVER_CSPARSE}
#   ${g2o_SOLVER_CHOLMOD}
#   ${g2o_TYPES_SLAM3D}
#   ${g2o_TYPES_SLAM3D_ADDONS}
# )

add_library(st_handeye_visp
  src/st_handeye_visp.cpp
)
target_link_libraries(st_handeye_visp
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${VISP_LIBRARIES}
)

# handeye_simulation
# add_executable(handeye_simulation
#   src/handeye_simulation.cpp
# )

target_link_libraries(
  st_handeye_graph
  g2o_core
  g2o_stuff
  g2o_types_slam3d
  g2o_types_slam3d_addons
  g2o_solver_dense
  g2o_solver_pcg
  g2o_solver_csparse
  g2o_solver_cholmod
  g2o_types_data
  ${G2O_INCLUDE_DIRS}
)
# add_dependencies(handeye_simulation
#   st_handeye_graph
#   st_handeye_visp
#)

# handeye_simulation_xray
# add_executable(handeye_simulation_xray
#   src/handeye_simulation_xray.cpp
# )
# target_link_libraries(handeye_simulation_xray
#     st_handeye_graph_xray
# )
# add_dependencies(handeye_simulation_xray
#   st_handeye_graph_xray
# )

# calibrate
# add_executable(calibrate
#   src/calibrate.cpp
# )
# target_link_libraries(calibrate
#     st_handeye_graph
#     st_handeye_visp
# )
# add_dependencies(calibrate
#   st_handeye_graph
#   st_handeye_visp
# )

# find_package(pybind11 CONFIG REQUIRED)
pybind11_add_module(_core MODULE src/core.cpp)

target_link_libraries(_core PUBLIC
  st_handeye_visp
  st_handeye_graph
)
target_include_directories(_core SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

install(TARGETS _core DESTINATION ${SKBUILD_PROJECT_NAME})