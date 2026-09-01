cmake_minimum_required(VERSION 3.0.2)
project(uwb_imu_pl)

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
  diagnostic_msgs
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
  IntegrityStatus.msg
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
    diagnostic_msgs
    uwb_driver
)

# --- GTSAM (≥ 4.2 required for GncOptimizer / setInlierCostThresholdsAtProbability) ---
find_package(GTSAM 4.2 REQUIRED)
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
  src/uwb_imu_pl/config/integrity_config.cpp
  src/uwb_imu_pl/io/run_logger.cpp
  src/uwb_imu_pl/integrity/estimation_snapshot.cpp
  src/uwb_imu_pl/estimation/snapshot_estimator.cpp
  src/uwb_imu_pl/integrity/integrity_monitor.cpp
  src/uwb_imu_pl/factors/realtime_factors.cpp
  src/uwb_imu_pl/estimation/incremental_estimator.cpp
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
# ROS Node: uwb_imu_pl_node
# =============================================
add_executable(uwb_imu_pl_node
  tools/run_offline.cpp
  src/visualizer.cpp
)

target_link_libraries(uwb_imu_pl_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam
  gtsam_unstable
  ${Boost_LIBRARIES}
  yaml-cpp
)

add_dependencies(uwb_imu_pl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

execute_process(
  COMMAND git rev-parse --short=12 HEAD
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

# =============================================
# Simulator — quadrotor dynamics + SO3 control
# =============================================
add_subdirectory(simulator)

catkin_install_python(PROGRAMS
  simulator/scripts/trajectory_circle.py
  simulator/scripts/trajectory_realtime.py
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

  catkin_add_gtest(test_snapshot_integrity test/test_snapshot_integrity.cpp)
  target_link_libraries(test_snapshot_integrity ${PROJECT_NAME} ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})

  catkin_add_gtest(test_realtime_incremental test/test_realtime_incremental.cpp)
  target_link_libraries(test_realtime_incremental ${PROJECT_NAME} gtsam ${Boost_LIBRARIES} yaml-cpp ${GTEST_LIBRARIES})
endif()
