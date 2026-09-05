#!/usr/bin/env bash

probe_package() {
  local package_name="$1"
  printf 'rospack find %s\n' "$package_name"
  rospack find "$package_name"
  printf 'exit_code=%d\n' "$?"
}

probe_environment() {
  printf 'CMAKE_PREFIX_PATH=%s\n' "${CMAKE_PREFIX_PATH:-UNSET}"
  printf 'ROS_PACKAGE_PATH=%s\n' "${ROS_PACKAGE_PATH:-UNSET}"
  printf 'LD_LIBRARY_PATH=%s\n' "${LD_LIBRARY_PATH:-UNSET}"
  printf 'PYTHONPATH=%s\n' "${PYTHONPATH:-UNSET}"
  probe_package uwb_imu_fgo
  probe_package uwb_driver
  probe_package isas_msgs
}

export ROS_HOME=/tmp/uwb_imu_fusion_t00_ros_home
mkdir -p "$ROS_HOME"

printf 'CURRENT_INHERITED_ENVIRONMENT\n'
probe_environment

printf 'OPT_ROS_NOETIC_ONLY\n'
(
  unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH ROSLISP_PACKAGE_DIRECTORIES
  source /opt/ros/noetic/setup.bash
  probe_environment
)

printf 'OLD_WS_OVERLAY\n'
(
  unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH ROSLISP_PACKAGE_DIRECTORIES
  source /opt/ros/noetic/setup.bash
  source /home/dev/ws_uwb/devel/setup.bash
  probe_environment
  printf 'resolved executable candidates\n'
  find /home/dev/ws_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo -maxdepth 1 -type f -printf '%p\n' 2>/dev/null | sort
)
