#!/usr/bin/env bash

unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash
cd /home/dev/ws_uwb_ie || exit 125

printf 'UTC=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
printf 'PWD=%s\n' "$PWD"
printf 'COMMAND=time catkin build uwb_imu_fgo --no-status\n'
printf 'CMAKE_PREFIX_PATH=%s\n' "${CMAKE_PREFIX_PATH:-UNSET}"
printf 'ROS_PACKAGE_PATH=%s\n' "${ROS_PACKAGE_PATH:-UNSET}"
printf 'LD_LIBRARY_PATH=%s\n' "${LD_LIBRARY_PATH:-UNSET}"
printf 'PYTHONPATH=%s\n' "${PYTHONPATH:-UNSET}"
printf 'SOURCE_HEAD=%s\n' "$(git -C /home/dev/ws_uwb_ie/src/uwb-imu-fusion rev-parse HEAD)"

time catkin build uwb_imu_fgo --no-status
build_status=$?
printf 'BUILD_EXIT_CODE=%d\n' "$build_status"
exit "$build_status"
