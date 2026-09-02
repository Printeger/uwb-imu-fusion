# ============================================================
# UWB-IMU FGO Simulator — quadrotor dynamics + SO(3) control
# Ported from ZJU-FAST-Lab/ego-planner-swarm
# ============================================================

# =============================================
# 1. quadrotor_dynamics — rigid-body + motor model
# =============================================
add_library(quadrotor_dynamics
  quadrotor_dynamics/src/Quadrotor.cpp
)
target_include_directories(quadrotor_dynamics PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/quadrotor_dynamics/include
  ${CMAKE_CURRENT_SOURCE_DIR}/quadrotor_dynamics/include/ode
)
target_link_libraries(quadrotor_dynamics
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
)

# =============================================
# 2. quadrotor_simulator_so3 — main simulation node
# =============================================
add_executable(quadrotor_simulator_so3
  quadrotor_simulator/src/quadrotor_simulator_so3.cpp
)
add_dependencies(quadrotor_simulator_so3
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadrotor_simulator_so3
  quadrotor_dynamics
  ${catkin_LIBRARIES}
)

# =============================================
# 3. SO3Control — geometric controller library
# =============================================
add_library(SO3Control
  so3_control/src/SO3Control.cpp
)

add_library(so3_control_nodelet
  so3_control/src/so3_control_nodelet.cpp
)
add_dependencies(so3_control_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(so3_control_nodelet
  SO3Control
  ${catkin_LIBRARIES}
)

# =============================================
# 4. uwb_twr_sim — UWB TWR simulation node
# =============================================
add_executable(uwb_twr_sim
  src/uwb_twr_sim.cpp
)
add_dependencies(uwb_twr_sim
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uwb_twr_sim
  ${catkin_LIBRARIES}
)
