#!/usr/bin/env bash

run_dir=/home/dev/ws_uwb_ie/src/uwb-imu-fusion/doc/ie_sprint/evidence/t00_20260905T102745Z/baseline_sfuise_walk1
binary_path=/home/dev/ws_uwb/devel/.private/uwb_imu_fgo/lib/uwb_imu_fgo/uwb_imu_fgo_node
config_path="$run_dir/config/config_effective.yaml"
input_path=/home/dev/ws_uwb/src/SFUISE/dataset/ISAS-Walk1.bag

unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash
source /home/dev/ws_uwb/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11322
export ROS_HOME="$run_dir/ros_home"

printf 'RUN_ID=t00_baseline_sfuise_walk1_old_cfe6d29\n'
printf 'UTC_START=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
printf 'PWD=%s\n' "$run_dir"
printf 'COMMAND=%s _config_path:=%s\n' "$binary_path" "$config_path"
printf 'ROS_MASTER_URI=%s\n' "$ROS_MASTER_URI"
printf 'CMAKE_PREFIX_PATH=%s\n' "$CMAKE_PREFIX_PATH"
printf 'ROS_PACKAGE_PATH=%s\n' "$ROS_PACKAGE_PATH"
printf 'LD_LIBRARY_PATH=%s\n' "$LD_LIBRARY_PATH"
printf 'OLD_SOURCE_HEAD=%s\n' "$(git -C /home/dev/ws_uwb/src/uwb-imu-fusion rev-parse HEAD)"
printf 'CURRENT_SOURCE_HEAD=%s\n' "$(git -C /home/dev/ws_uwb_ie/src/uwb-imu-fusion rev-parse HEAD)"
sha256sum "$binary_path" "$config_path" "$run_dir/config/config_original.yaml" "$input_path"
stat -c 'BINARY_STAT=%n size=%s mtime=%y' "$binary_path"

roscore -p 11322 >"$run_dir/roscore.log" 2>&1 &
roscore_pid=$!
cleanup() {
  if kill -0 "$roscore_pid" 2>/dev/null; then
    kill -INT "$roscore_pid" 2>/dev/null
    wait "$roscore_pid" 2>/dev/null
  fi
}
trap cleanup EXIT

master_ready=0
for _ in $(seq 1 100); do
  if rosparam list >/dev/null 2>&1; then
    master_ready=1
    break
  fi
  sleep 0.1
done
printf 'ROS_MASTER_READY=%d\n' "$master_ready"
if [[ "$master_ready" -ne 1 ]]; then
  exit 125
fi

cd "$run_dir" || exit 125
start_ns=$(date +%s%N)
stdbuf -oL -eL "$binary_path" "_config_path:=$config_path" >"$run_dir/stdout.log" 2>"$run_dir/stderr.log" &
node_pid=$!
peak_rss_kib=0
termination=NATURAL_EXIT
polls=0

while kill -0 "$node_pid" 2>/dev/null; do
  rss_kib=$(awk '/^VmRSS:/ {print $2}' "/proc/$node_pid/status" 2>/dev/null)
  if [[ "$rss_kib" =~ ^[0-9]+$ ]] && (( rss_kib > peak_rss_kib )); then
    peak_rss_kib=$rss_kib
  fi
  if grep -q 'Visualization published' "$run_dir/stdout.log" 2>/dev/null; then
    termination=SIGINT_AFTER_VISUALIZATION_OUTPUT
    kill -INT "$node_pid"
    break
  fi
  ((polls += 1))
  if (( polls >= 1200 )); then
    termination=SIGINT_AFTER_120_SECONDS
    kill -INT "$node_pid"
    break
  fi
  sleep 0.1
done

wait "$node_pid"
node_status=$?
end_ns=$(date +%s%N)
elapsed_ns=$((end_ns - start_ns))
elapsed_seconds=$(awk -v ns="$elapsed_ns" 'BEGIN {printf "%.3f", ns / 1000000000}')

printf 'TERMINATION=%s\n' "$termination"
printf 'NODE_EXIT_CODE=%d\n' "$node_status"
printf 'WALL_SECONDS=%s\n' "$elapsed_seconds"
printf 'PEAK_RSS_KIB=%d\n' "$peak_rss_kib"
printf 'UTC_END=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
printf 'OUTPUT_FILES\n'
find "$run_dir" -maxdepth 3 -type f -printf '%s\t%p\n' | sort -k2
printf 'STDOUT_TAIL\n'
tail -n 100 "$run_dir/stdout.log"
printf 'STDERR_TAIL\n'
tail -n 100 "$run_dir/stderr.log"
exit "$node_status"
