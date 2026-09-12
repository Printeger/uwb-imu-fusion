cmake_minimum_required(VERSION 3.0.2)
project(sfuise_baseline_adapter)

add_compile_options(-std=c++17 -O2)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs
  nav_msgs visualization_msgs isas_msgs sfuise_msgs)
find_package(Eigen3 REQUIRED)

catkin_package()
if(NOT DEFINED SFUISE_SOURCE_DIR)
  message(FATAL_ERROR "SFUISE_SOURCE_DIR must identify the unmodified SFUISE checkout")
endif()
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  ${SFUISE_SOURCE_DIR}/sfuise/include)

add_executable(sfuise_trajectory_adapter src/sfuise_trajectory_adapter.cpp)
add_dependencies(sfuise_trajectory_adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(sfuise_trajectory_adapter ${catkin_LIBRARIES})
