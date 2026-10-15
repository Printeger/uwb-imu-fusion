cmake_minimum_required(VERSION 3.0.2)
project(uwb_benchmark_sfuise_adapter)

find_package(catkin REQUIRED COMPONENTS roscpp isas_msgs sfuise_msgs std_msgs rosgraph_msgs rosbag)
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

# Optional SC8 copy of the pinned source with observation-only instrumentation.
# The upstream checkout remains read-only; the generator seals its exact patch.
if(SFUISE_SC8_OBSERVER_SOURCE)
  list(APPEND CMAKE_MODULE_PATH ${SFUISE_INCLUDE_DIR}/../cmake_modules)
  find_package(SuiteSparse REQUIRED MODULE)
  add_executable(sfuise_sc8_observed ${SFUISE_SC8_OBSERVER_SOURCE})
  add_dependencies(sfuise_sc8_observed ${catkin_EXPORTED_TARGETS})
  target_include_directories(sfuise_sc8_observed PRIVATE ${SUITESPARSE_INCLUDE_DIRS})
  target_compile_features(sfuise_sc8_observed PRIVATE cxx_std_17)
  target_compile_options(sfuise_sc8_observed PRIVATE -O3 -DNDEBUG)
  target_link_libraries(sfuise_sc8_observed ${catkin_LIBRARIES} cholmod)
  get_filename_component(SC8_OBSERVER_DIR ${SFUISE_SC8_OBSERVER_SOURCE} DIRECTORY)
  add_executable(sfuise_sc8_observer_checks ${SC8_OBSERVER_DIR}/checks.cpp)
  add_dependencies(sfuise_sc8_observer_checks ${catkin_EXPORTED_TARGETS})
  target_include_directories(sfuise_sc8_observer_checks PRIVATE ${SUITESPARSE_INCLUDE_DIRS})
  target_compile_options(sfuise_sc8_observer_checks PRIVATE -O3 -DNDEBUG)
  target_compile_features(sfuise_sc8_observer_checks PRIVATE cxx_std_17)
  target_link_libraries(sfuise_sc8_observer_checks ${catkin_LIBRARIES} cholmod)
endif()
