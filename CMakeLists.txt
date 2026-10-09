cmake_minimum_required(VERSION 3.0.2)
project(uwb_imu_pl)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
add_compile_options(-Wall -Wno-sign-compare -Wno-unused -Wno-comment -pthread)

# --- Catkin ---
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  nodelet
  message_generation
  visualization_msgs
  diagnostic_msgs
)

add_message_files(
  FILES
  LinktrackNode2.msg
  LinktrackNodeframe3.msg
  AuxCommand.msg
  Corrections.msg
  PositionCommand.msg
  SO3Command.msg
  IntegrityStatus.msg
  NavigationIntegrity.msg
  FaultTruth.msg
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
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    message_runtime
    visualization_msgs
    diagnostic_msgs
)

# --- GTSAM (≥ 4.2; unstable provides IncrementalFixedLagSmoother) ---
find_package(GTSAM 4.2 REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
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

if(CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
endif()

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
  src/uwb_imu_pl/common/types.cpp
  src/uwb_imu_pl/common/integrity_ids.cpp
  src/uwb_imu_pl/common/failure_reason.cpp
  src/uwb_imu_pl/common/integrity_identity.cpp
  src/uwb_imu_pl/config/integrity_config.cpp
  src/uwb_imu_pl/config/fault_manifest.cpp
  src/uwb_imu_pl/io/run_logger.cpp
  src/uwb_imu_pl/integrity/estimation_snapshot.cpp
  src/uwb_imu_pl/integrity/fault_scope.cpp
  src/uwb_imu_pl/estimation/snapshot_estimator.cpp
  src/uwb_imu_pl/integrity/integrity_monitor.cpp
  src/uwb_imu_pl/integrity/dense_oracle.cpp
  src/uwb_imu_pl/factors/realtime_factors.cpp
  src/uwb_imu_pl/estimation/incremental_estimator.cpp
  src/uwb_imu_pl/estimation/estimation_tuning.cpp
  src/uwb_imu_pl/estimation/causal_initializer.cpp
  src/uwb_imu_pl/estimation/factor_ledger.cpp
  src/uwb_imu_pl/estimation/integrity_window_snapshot.cpp
  src/uwb_imu_pl/estimation/square_root_context.cpp
  src/uwb_imu_pl/integrity/numerical_phase_profile.cpp
  src/uwb_imu_pl/estimation/candidate_replay.cpp
  src/uwb_imu_pl/estimation/candidate_worker_pool.cpp
  src/uwb_imu_pl/estimation/rank_update_kernel.cpp
  src/uwb_imu_pl/estimation/reinitialization.cpp
  src/uwb_imu_pl/integrity/joint_window_detector.cpp
  src/uwb_imu_pl/integrity/hypothesis_evidence.cpp
  src/uwb_imu_pl/integrity/attempt_proof_arena.cpp
  src/uwb_imu_pl/integrity/hypothesis_generator.cpp
  src/uwb_imu_pl/integrity/coverage_envelope.cpp
  src/uwb_imu_pl/integrity/fde_manager.cpp
  src/uwb_imu_pl/integrity/risk_budget_audit.cpp
  src/uwb_imu_pl/integrity/protection_level_v2.cpp
  src/uwb_imu_pl/integrity/statistical_bounds_cache.cpp
  src/uwb_imu_pl/integrity/health_manager.cpp
  src/uwb_imu_pl/integrity/imu_fault_subspace.cpp
  src/uwb_imu_pl/integrity/history_fault_summary.cpp
  src/uwb_imu_pl/integrity/history_fault_parameterization.cpp
  src/uwb_imu_pl/integrity/history_summary_extraction.cpp
  src/uwb_imu_pl/integrity/dual_channel_detector.cpp
  src/uwb_imu_pl/integrity/fde_post_selection.cpp
  src/uwb_imu_pl/integrity/publication_identity.cpp
  src/uwb_imu_pl/publication/final_output_packet.cpp
  src/uwb_imu_pl/integrity/history_summary_lifecycle.cpp
  src/uwb_imu_pl/factors/kinematic_bridge_factor.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${Boost_LIBRARIES}
  yaml-cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

execute_process(
  COMMAND git rev-parse HEAD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE UWB_IMU_PL_GIT_SHA
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
execute_process(
  COMMAND git status --porcelain
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE UWB_IMU_PL_GIT_STATUS
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(UWB_IMU_PL_GIT_STATUS STREQUAL "")
  set(UWB_IMU_PL_GIT_DIRTY 0)
else()
  set(UWB_IMU_PL_GIT_DIRTY 1)
endif()

add_executable(uwb_imu_pl_realtime_node tools/run_realtime_integrity.cpp)
target_compile_definitions(uwb_imu_pl_realtime_node PRIVATE
  UWB_IMU_PL_GIT_SHA="${UWB_IMU_PL_GIT_SHA}"
  UWB_IMU_PL_GIT_DIRTY=${UWB_IMU_PL_GIT_DIRTY})
target_link_libraries(uwb_imu_pl_realtime_node
  ${PROJECT_NAME} ${catkin_LIBRARIES} gtsam ${Boost_LIBRARIES} yaml-cpp)
add_dependencies(uwb_imu_pl_realtime_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(snapshot_integrity_sweep apps/snapshot_integrity_sweep.cpp)
target_link_libraries(snapshot_integrity_sweep
  ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp)

add_executable(integrity_monte_carlo apps/integrity_monte_carlo.cpp)
target_compile_definitions(integrity_monte_carlo PRIVATE
  UWB_IMU_PL_GIT_SHA="${UWB_IMU_PL_GIT_SHA}"
  UWB_IMU_PL_GIT_DIRTY=${UWB_IMU_PL_GIT_DIRTY})
target_link_libraries(integrity_monte_carlo
  ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp)

add_executable(method_ab_shadow apps/method_ab_shadow.cpp)
target_link_libraries(method_ab_shadow
  ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp)

add_executable(sequential_detector_history apps/sequential_detector_history.cpp)
target_link_libraries(sequential_detector_history
  ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp)

add_executable(realtime_performance_benchmark apps/realtime_performance_benchmark.cpp)
target_compile_definitions(realtime_performance_benchmark PRIVATE
  UWB_IMU_PL_GIT_SHA="${UWB_IMU_PL_GIT_SHA}"
  UWB_IMU_PL_GIT_DIRTY=${UWB_IMU_PL_GIT_DIRTY})
target_link_libraries(realtime_performance_benchmark
  ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp)

add_executable(r0_r1_development apps/r0_r1_development.cpp)
target_compile_definitions(r0_r1_development PRIVATE
  UWB_IMU_PL_GIT_SHA="${UWB_IMU_PL_GIT_SHA}"
  UWB_IMU_PL_GIT_DIRTY=${UWB_IMU_PL_GIT_DIRTY})
target_link_libraries(r0_r1_development
  ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp)

add_executable(gate_d_development apps/gate_d_development.cpp)
target_link_libraries(gate_d_development ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp)

add_executable(candidate_replay apps/candidate_replay.cpp)
target_link_libraries(candidate_replay ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp)

add_executable(integrity_round2_scenario apps/integrity_round2_scenario.cpp)
target_link_libraries(integrity_round2_scenario
  ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp)

add_executable(advisor_paired_benchmark apps/advisor_paired_benchmark.cpp)
target_link_libraries(advisor_paired_benchmark
  ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp)

# Dataset benchmark: direct nominal estimator path.  This executable does not
# construct the integrity pipeline, detector, FDE, bridge, or PL components.
add_executable(nominal_dataset_runner apps/nominal_dataset_runner.cpp)
target_compile_definitions(nominal_dataset_runner PRIVATE
  UWB_IMU_PL_GIT_SHA="${UWB_IMU_PL_GIT_SHA}"
  UWB_IMU_PL_GIT_DIRTY=${UWB_IMU_PL_GIT_DIRTY})
target_link_libraries(nominal_dataset_runner
  ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp)

# =============================================
# Simulator — quadrotor dynamics + SO3 control
# =============================================
add_subdirectory(simulator)

catkin_install_python(PROGRAMS
  simulator/scripts/trajectory_realtime.py
  simulator/scripts/realtime_integrity_viz.py
  tools/validate_run_schema.py
  tools/generate_integrity_report.py
  tools/summarize_snapshot_sweep.py
  tools/calibrate_detector_roc.py
  tools/analyze_noncentral.py
  tools/analyze_detector_roc.py
  tools/run_imu_bias_monte_carlo.py
  tools/week4_topic_publisher.py
  tools/validate_week4_ros.py
  tools/analyze_week4_performance.py
  tools/week4_common.py
  tools/run_week4_validation.py
  tools/round2_common.py
  tools/run_integrity_round2.py
  tools/run_round2_analytic_mc.py
  tools/summarize_gate_d_development.py
  tools/gate_d_diagnostics.py
  tools/run_round2_performance.py
  tools/run_integrity_round3.py
  tools/summarize_gate_d_p3_p6.py
  tools/generate_fde_profile_configs.py
  tools/run_fde_profile_validation.py
  tools/analyze_fde_profile_runs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# =============================================
# Unit Tests (GoogleTest)
# =============================================
if(CATKIN_ENABLE_TESTING)
  enable_testing()
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_snapshot_integrity test/test_snapshot_integrity.cpp test/test_main.cpp)
  target_link_libraries(test_snapshot_integrity ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_integrity_config test/test_integrity_config.cpp test/test_main.cpp)
  target_compile_definitions(test_integrity_config PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_integrity_config ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_run_logger test/test_run_logger.cpp test/test_main.cpp)
  target_link_libraries(test_run_logger ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_nominal_accuracy test/test_nominal_accuracy.cpp test/test_main.cpp)
  target_link_libraries(test_nominal_accuracy ${PROJECT_NAME} gtsam yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_realtime_incremental test/test_realtime_incremental.cpp test/test_main.cpp)
  target_link_libraries(test_realtime_incremental ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_deterministic_event test/test_deterministic_event.cpp test/test_main.cpp)
  target_link_libraries(test_deterministic_event ${PROJECT_NAME} ${GTEST_LIBRARIES})

  catkin_add_gtest(test_dense_oracle test/test_dense_oracle.cpp test/test_main.cpp)
  target_link_libraries(test_dense_oracle ${PROJECT_NAME} ${GTEST_LIBRARIES})

  catkin_add_gtest(test_integrity_v2 test/test_integrity_v2.cpp test/test_main.cpp)
  target_compile_definitions(test_integrity_v2 PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_integrity_v2 ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_protection_mode_cache test/test_protection_mode_cache.cpp test/test_main.cpp)
  target_link_libraries(test_protection_mode_cache ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_integrity_reference test/test_integrity_reference.cpp test/test_main.cpp)
  target_compile_definitions(test_integrity_reference PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_integrity_reference ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_fault_manifest test/test_fault_manifest.cpp test/test_main.cpp)
  target_compile_definitions(test_fault_manifest PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_fault_manifest ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_square_root_context test/test_square_root_context.cpp test/test_main.cpp)
  target_compile_definitions(test_square_root_context PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_square_root_context ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_history_fault_summary test/test_history_fault_summary.cpp test/test_main.cpp)
  target_link_libraries(test_history_fault_summary ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_history_fault_parameterization test/test_history_fault_parameterization.cpp test/test_main.cpp)
  target_compile_definitions(test_history_fault_parameterization PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_history_fault_parameterization ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_history_summary_extraction test/test_history_summary_extraction.cpp test/test_main.cpp)
  target_compile_definitions(test_history_summary_extraction PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_history_summary_extraction ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_history_summary_lifecycle test/test_history_summary_lifecycle.cpp test/test_main.cpp)
  target_compile_definitions(test_history_summary_lifecycle PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_history_summary_lifecycle ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_dual_channel_detector test/test_dual_channel_detector.cpp test/test_main.cpp)
  target_compile_definitions(test_dual_channel_detector PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_dual_channel_detector ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_fde_post_selection test/test_fde_post_selection.cpp test/test_main.cpp)
  target_compile_definitions(test_fde_post_selection PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_fde_post_selection ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_p0_04_risk_oracle test/test_p0_04_risk_oracle.cpp test/test_main.cpp)
  target_compile_definitions(test_p0_04_risk_oracle PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_p0_04_risk_oracle ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_publication_identity test/test_publication_identity.cpp test/test_main.cpp)
  target_compile_definitions(test_publication_identity PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_publication_identity ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_publication_wiring test/test_publication_wiring.cpp test/test_main.cpp)
  target_compile_definitions(test_publication_wiring PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_publication_wiring ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_p0_05_publication test/test_p0_05_publication.cpp test/test_main.cpp)
  target_compile_definitions(test_p0_05_publication PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_p0_05_publication ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_p0_06_action_search test/test_p0_06_action_search.cpp test/test_main.cpp)
  target_link_libraries(test_p0_06_action_search ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_p0_07_corrected_exhaustive test/test_p0_07_corrected_exhaustive.cpp test/test_main.cpp)
  target_compile_definitions(test_p0_07_corrected_exhaustive PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_p0_07_corrected_exhaustive ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  add_executable(p0_07_production_probe tools/p0_07_production_probe.cpp)
  target_link_libraries(p0_07_production_probe
    ${PROJECT_NAME} gtsam gtsam_unstable ${Boost_LIBRARIES} yaml-cpp)

  add_test(NAME test_p0_07_process_oracle
    COMMAND ${PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/run_p0_07_process_oracle.py
      --probe $<TARGET_FILE:p0_07_production_probe>
      --source ${CMAKE_CURRENT_SOURCE_DIR})

  # D12 acceptance is the complete, process-isolated 174-leaf protocol.  Keep
  # it as an ordinary CTest so a complete-suite result cannot silently omit
  # the expensive mutation coverage that was previously run by hand.
  add_test(NAME test_p0_07_leaf_protocol
    COMMAND ${PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/test_p0_07_leaf_protocol.py
      --full
      --probe $<TARGET_FILE:p0_07_production_probe>
      --source ${CMAKE_CURRENT_SOURCE_DIR})
  set_tests_properties(test_p0_07_leaf_protocol PROPERTIES TIMEOUT 3600)

  add_test(NAME test_p0_07_evidence_runner
    COMMAND ${PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_p0_07_evidence.py
      --binary $<TARGET_FILE:test_p0_07_corrected_exhaustive>
      --input ${CMAKE_CURRENT_SOURCE_DIR}/docs/evidence/p0-07-corrected-exhaustive/authoritative-replay/frozen-input.json
      --reference ${CMAKE_CURRENT_SOURCE_DIR}/docs/evidence/p0-07-corrected-exhaustive/authoritative-replay/frozen-truth.json
      --compare-dir ${CMAKE_CURRENT_SOURCE_DIR}/docs/evidence/p0-07-corrected-exhaustive/authoritative-replay
      --output-dir ${CMAKE_CURRENT_BINARY_DIR}/p0-07-replay-bundle)
  add_test(NAME test_p0_07_evidence_runner_atomic_failure
    COMMAND ${PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/tools/test_p0_07_runner_atomic.py
      --runner ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_p0_07_evidence.py
      --binary $<TARGET_FILE:test_p0_07_corrected_exhaustive>
      --input ${CMAKE_CURRENT_SOURCE_DIR}/docs/evidence/p0-07-corrected-exhaustive/authoritative-replay/frozen-input.json
      --reference ${CMAKE_CURRENT_SOURCE_DIR}/docs/evidence/p0-07-corrected-exhaustive/authoritative-replay/frozen-truth.json
      --canonical ${CMAKE_CURRENT_SOURCE_DIR}/docs/evidence/p0-07-corrected-exhaustive/authoritative-replay)

  add_executable(p005_ros_certified_publisher test/p005_ros_certified_publisher.cpp)
  target_link_libraries(p005_ros_certified_publisher ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${catkin_LIBRARIES})

  catkin_add_gtest(test_history_summary_pipeline test/test_history_summary_pipeline.cpp test/test_main.cpp)
  target_compile_definitions(test_history_summary_pipeline PRIVATE
    UWB_IMU_PL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(test_history_summary_pipeline ${PROJECT_NAME} gtsam gtsam_unstable ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  add_rostest(test/realtime_integrity_viz.test)

  add_test(NAME test_week4_tools
    COMMAND ${CMAKE_COMMAND} -E env
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools
      python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_week4_tools.py)

  add_test(NAME test_advisor_report
    COMMAND ${CMAKE_COMMAND} -E env
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools
      python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_advisor_report.py)

  add_test(NAME test_round2_tools
    COMMAND ${CMAKE_COMMAND} -E env
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools
      python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_round2_tools.py)

  add_test(NAME test_gate_d_tools
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools
      python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gate_d_tools.py)

  add_test(NAME test_simulation_calibrations
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools
      python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_simulation_calibrations.py)

  add_test(NAME test_round3_tools
    COMMAND ${CMAKE_COMMAND} -E env
      PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools
      python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_round3_tools.py)

  add_test(NAME test_launch_contracts
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch_contracts.py)

  add_test(NAME test_dataset_benchmark
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/tools
      python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_dataset_benchmark.py)
endif()
