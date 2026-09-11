cmake_minimum_required(VERSION 3.0.2)
project(uwb_imu_fgo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -Wno-sign-compare -Wno-unused -Wno-comment -pthread")

# --- Catkin ---
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  nodelet
  message_generation
  eigen_conversions
  visualization_msgs
  uwb_driver
  isas_msgs
)

add_message_files(
  FILES
  LinktrackNode2.msg
  LinktrackNodeframe3.msg
  McdLinktrackNodeframe3.msg
  LinktrackTagframe0.msg
  AuxCommand.msg
  Corrections.msg
  Gains.msg
  LQRTrajectory.msg
  Odometry.msg
  OutputData.msg
  PPROutputData.msg
  PolynomialTrajectory.msg
  PositionCommand.msg
  SO3Command.msg
  Serial.msg
  StatusData.msg
  TRPYCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rosbag
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    message_runtime
    eigen_conversions
    visualization_msgs
    uwb_driver
)

# --- GTSAM (≥ 4.2 required for GncOptimizer / setInlierCostThresholdsAtProbability) ---
find_package(GTSAM 4.2 REQUIRED)
# Bind unstable to the same installation instead of a system -l search that
# can silently mix incompatible GTSAM ABIs.
find_package(GTSAM_UNSTABLE REQUIRED CONFIG
  PATHS "${GTSAM_DIR}/../GTSAM_UNSTABLE" NO_DEFAULT_PATH)
if(GTSAM_FOUND)
  include_directories(${GTSAM_INCLUDE_DIR})
  message(STATUS "GTSAM found: ${GTSAM_INCLUDE_DIR} (version: ${GTSAM_VERSION})")
endif()

# --- Eigen3 ---
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

# --- yaml-cpp ---
find_package(yaml-cpp REQUIRED)

# --- Boost ---
find_package(Boost REQUIRED COMPONENTS filesystem iostreams program_options system serialization thread)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
  link_directories(${Boost_LIBRARY_DIRS})
endif()

# --- GoogleTest ---
find_package(GTest REQUIRED)
find_package(PythonInterp REQUIRED)

# --- Armadillo (simulator: ODE integrator) ---
find_package(Armadillo REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${ARMADILLO_INCLUDE_DIRS}
  simulator/quadrotor_dynamics/include
  simulator/uav_utils/include
  simulator/so3_control/include
)

# =============================================
# Library: core algorithm (ROS-free)
# =============================================
add_library(${PROJECT_NAME}
  src/config.cpp
  src/data_loader.cpp
  src/outlier_filter.cpp
  src/initializer.cpp
  src/imu_preint.cpp
  src/imu_sync.cpp
  src/uwb_factor.cpp
  src/graph_builder.cpp
  src/optimizer.cpp
  src/chi2.cpp
  src/trajectory_io.cpp
  src/logger.cpp
  src/hash_utils.cpp
  src/paper_input.cpp
  src/t07_scenario_cache.cpp
  src/nlos_solver_utils.cpp
  src/nlos_fde.cpp
  src/fde_math.cpp
  src/fde_grouped.cpp
  src/nlos_discovery.cpp
  src/nlos_refit.cpp
  src/nlos_recoverability.cpp
  src/nlos_scoring.cpp
  src/nlos_inference.cpp
  src/nlos_inference_io.cpp
  src/paper_methods.cpp
  src/paper_run_io.cpp
  src/paper_stage2_cache.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${Boost_LIBRARIES}
  yaml-cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# =============================================
# ROS Node: uwb_imu_fgo_node
# =============================================
add_executable(uwb_imu_fgo_node
  tools/run_offline.cpp
  src/visualizer.cpp
)

target_link_libraries(uwb_imu_fgo_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${Boost_LIBRARIES}
  yaml-cpp
)

add_dependencies(uwb_imu_fgo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Thin batch entry for the paper input path. It intentionally bypasses the
# legacy Optimizer GNC/rejection loop; it supports the T02 all-range graph and
# the T04 oracle-support debug refit.
add_executable(uwb_imu_fgo_paper_runner tools/run_ie_paper.cpp)
target_link_libraries(uwb_imu_fgo_paper_runner
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${Boost_LIBRARIES}
  yaml-cpp
)
add_dependencies(uwb_imu_fgo_paper_runner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(export_ground_truth tools/paper/export_ground_truth.cpp)
target_link_libraries(export_ground_truth
  ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam ${Boost_LIBRARIES} yaml-cpp)
add_dependencies(export_ground_truth ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# T07 generation code is deliberately isolated from the estimator library and
# paper runner. The runner can only read the self-contained estimator cache.
add_library(t07_scenario_generation
  tools/paper/t07_scenario_injector.cpp
)
target_link_libraries(t07_scenario_generation
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
add_dependencies(t07_scenario_generation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(generate_t07_scenario tools/paper/generate_t07_scenario.cpp)
target_link_libraries(generate_t07_scenario
  t07_scenario_generation ${PROJECT_NAME} ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} yaml-cpp)
add_dependencies(generate_t07_scenario ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(t06_chain_solver_probe tools/paper/t06_chain_solver_probe.cpp)
target_link_libraries(t06_chain_solver_probe ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam yaml-cpp)
add_dependencies(t06_chain_solver_probe ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# R04 keeps the development-only certified runner inside the project build so
# header-defined Eigen/GTSAM factor types use the same compile ABI as core.
add_executable(a19_r04_pipeline tools/paper/a19_r04_pipeline.cpp)
target_link_libraries(a19_r04_pipeline
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
  ${Boost_LIBRARIES}
  yaml-cpp
  /usr/lib/x86_64-linux-gnu/libmpfr.so.6
  /usr/lib/x86_64-linux-gnu/libgmp.so.10
)
add_dependencies(a19_r04_pipeline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Opt-in R08 validation envelope. It shares the production estimator library
# and certified implementation; the separate executable keeps legacy R04/R07
# entry modes and identities unchanged.
add_executable(a19_r08_pipeline tools/paper/a19_r08_pipeline.cpp)
target_link_libraries(a19_r08_pipeline
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
  ${Boost_LIBRARIES}
  yaml-cpp
  /usr/lib/x86_64-linux-gnu/libmpfr.so.6
  /usr/lib/x86_64-linux-gnu/libgmp.so.10
)
add_dependencies(a19_r08_pipeline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# =============================================
# Simulator — quadrotor dynamics + SO3 control
# =============================================
add_subdirectory(simulator)

catkin_install_python(PROGRAMS
  simulator/scripts/trajectory_circle.py
  simulator/scripts/odom_to_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# =============================================
# Unit Tests (GoogleTest)
# =============================================
if(GTest_FOUND)
  enable_testing()

  catkin_add_gtest(test_config test/test_config.cpp)
  target_link_libraries(test_config ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_outlier test/test_outlier.cpp)
  target_link_libraries(test_outlier ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_trilaterate test/test_trilaterate.cpp)
  target_link_libraries(test_trilaterate ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_imu_preint test/test_imu_preint.cpp)
  target_link_libraries(test_imu_preint ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_uwb_factor test/test_uwb_factor.cpp)
  target_link_libraries(test_uwb_factor ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_graph_builder test/test_graph_builder.cpp)
  target_link_libraries(test_graph_builder ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_optimizer test/test_optimizer.cpp)
  target_link_libraries(test_optimizer ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_paper_input test/test_paper_input.cpp)
  target_link_libraries(test_paper_input ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_t07_scenario test/test_t07_scenario.cpp)
  target_link_libraries(test_t07_scenario t07_scenario_generation ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_nlos_refit test/test_nlos_refit.cpp)
  target_link_libraries(test_nlos_refit ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_nlos_discovery test/test_nlos_discovery.cpp)
  target_link_libraries(test_nlos_discovery ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_nlos_fde test/test_nlos_fde.cpp)
  target_link_libraries(test_nlos_fde ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_nlos_recoverability test/test_nlos_recoverability.cpp)
  target_link_libraries(test_nlos_recoverability ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})
  target_compile_definitions(test_nlos_recoverability PRIVATE
    UIFGO_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

  catkin_add_gtest(test_nlos_inference test/test_nlos_inference.cpp)
  target_link_libraries(test_nlos_inference ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_paper_methods test/test_paper_methods.cpp)
  target_link_libraries(test_paper_methods ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_paper_stage2_cache test/test_paper_stage2_cache.cpp)
  target_link_libraries(test_paper_stage2_cache ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable yaml-cpp ${GTEST_LIBRARIES})

  add_test(NAME test_fde_runner_contract
    COMMAND ${PYTHON_EXECUTABLE} -B
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_fde_runner_contract.py)

  add_test(NAME test_t04_runner_contract
    COMMAND ${PYTHON_EXECUTABLE} -B
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_t04_runner_contract.py
      --runner $<TARGET_FILE:uwb_imu_fgo_paper_runner>
      --source-config
        ${CMAKE_CURRENT_SOURCE_DIR}/config/paper/sim_circle_t04_oracle_debug.yaml
      --bag
        ${CMAKE_CURRENT_SOURCE_DIR}/data/sim_circle_2026-06-15-16-04-35.bag)

  add_test(NAME test_t06_chain_reference
    COMMAND ${PYTHON_EXECUTABLE} -B
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_t06_chain_reference.py
      --solver $<TARGET_FILE:t06_chain_solver_probe>)

  add_test(NAME test_t06_runner_contract
    COMMAND ${PYTHON_EXECUTABLE} -B
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_t06_runner_contract.py
      --runner $<TARGET_FILE:uwb_imu_fgo_paper_runner>
      --source-config
        ${CMAKE_CURRENT_SOURCE_DIR}/config/paper/sim_circle_t06_automatic_discovery.yaml
      --bag
        ${CMAKE_CURRENT_SOURCE_DIR}/data/sim_circle_2026-06-15-16-04-35.bag)

  add_test(NAME test_t08_runner_contract
    COMMAND ${PYTHON_EXECUTABLE} -B
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_t08_runner_contract.py
      --runner $<TARGET_FILE:uwb_imu_fgo_paper_runner>
      --config
        ${CMAKE_CURRENT_SOURCE_DIR}/config/paper/sim_circle_t08_oracle_development.yaml)

  add_test(NAME test_t09_runner_contract
    COMMAND ${PYTHON_EXECUTABLE} -B
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_t09_runner_contract.py)

  add_test(NAME test_ie0911_evaluator
    COMMAND ${PYTHON_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ie0911_evaluator.py)
  add_test(NAME test_ie0911_step2
    COMMAND ${PYTHON_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/test/test_ie0911_step2.py)

  add_test(NAME test_t09_evaluator
    COMMAND ${PYTHON_EXECUTABLE} -B
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_t09_evaluator.py)
  add_test(NAME test_t09_review_fix
    COMMAND ${PYTHON_EXECUTABLE} -B
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_t09_review_fix.py)
  add_test(NAME test_t09_evaluator_artifact_graph
    COMMAND ${PYTHON_EXECUTABLE} -B
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_t09_evaluator_artifact_graph.py)
  add_test(NAME test_t09_comparability_boundary
    COMMAND ${PYTHON_EXECUTABLE} -B
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_t09_comparability_boundary.py)
  add_test(NAME test_t09_rereview_fix
    COMMAND ${PYTHON_EXECUTABLE} -B
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_t09_rereview_fix.py)
endif()

# Experiment-only export; algorithm library is shared with the paper runner.
add_executable(export_sfuise_input tools/paper/export_sfuise_input.cpp)
target_link_libraries(export_sfuise_input ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam yaml-cpp)
add_dependencies(export_sfuise_input ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
 add_test(NAME test_nlos_injection COMMAND ${PYTHON_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/test/test_nlos_injection.py)
endif()

# Truth-derived hold-outs are accepted only by this independent forensic binary.
add_executable(uwb_imu_fgo_fde_forensics tools/run_fde_forensics.cpp)
target_link_libraries(uwb_imu_fgo_fde_forensics ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable ${Boost_LIBRARIES} yaml-cpp)
add_dependencies(uwb_imu_fgo_fde_forensics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_fde_math test/test_fde_math.cpp)
  target_link_libraries(test_fde_math ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam gtsam_unstable ${GTEST_LIBRARIES})
endif()
