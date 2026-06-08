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
  message_generation
  eigen_conversions
  visualization_msgs
)

add_message_files(
  FILES
  LinktrackNode2.msg
  LinktrackNodeframe3.msg
  LinktrackTagframe0.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
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
)

# --- GTSAM ---
find_package(GTSAM REQUIRED)
if(GTSAM_FOUND)
  include_directories(${GTSAM_INCLUDE_DIR})
  message(STATUS "GTSAM found: ${GTSAM_INCLUDE_DIR}")
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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
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
endif()
