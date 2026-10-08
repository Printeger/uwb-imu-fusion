cmake_minimum_required(VERSION 3.0.2)
project(uwb_benchmark_sfuise_adapter)

find_package(catkin REQUIRED COMPONENTS roscpp isas_msgs sfuise_msgs std_msgs)
find_package(Eigen3 REQUIRED)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${SFUISE_INCLUDE_DIR}
)

add_executable(sfuise_trajectory_adapter src/sfuise_trajectory_adapter.cpp)
add_dependencies(sfuise_trajectory_adapter ${catkin_EXPORTED_TARGETS})
target_compile_features(sfuise_trajectory_adapter PRIVATE cxx_std_17)
target_link_libraries(sfuise_trajectory_adapter ${catkin_LIBRARIES})
