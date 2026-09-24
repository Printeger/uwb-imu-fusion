#!/usr/bin/env bash
set -euo pipefail

source /home/mint/ws_fusion_uwb/devel/setup.bash

evidence_dir=/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl/docs/evidence/p0-05-publication
run_dir=/tmp/p005_ros_certified
capture_file=/tmp/p005_ros_certified_capture.json
publisher=/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/p005_ros_certified_publisher

test ! -e "${run_dir}" || mv "${run_dir}" "${run_dir}.previous.$(date +%s)"

roscore > /tmp/p005_ros_certified_roscore.log 2>&1 &
master_pid=$!
capture_pid=
cleanup() {
  if test -n "${capture_pid}"; then
    kill -INT "${capture_pid}" 2>/dev/null || true
    wait "${capture_pid}" 2>/dev/null || true
  fi
  kill -INT "${master_pid}" 2>/dev/null || true
  wait "${master_pid}" 2>/dev/null || true
}
trap cleanup EXIT

for unused in $(seq 1 100); do
  if rosparam list >/dev/null 2>&1; then break; fi
  sleep 0.05
done

python3 "${evidence_dir}/capture_ros_publication.py" \
  --require-committed --timeout 30 --output "${capture_file}" \
  --csv "${run_dir}/integrity.csv" \
  > /tmp/p005_ros_certified_capture.log 2>&1 &
capture_pid=$!
sleep 0.5
"${publisher}" _run_directory:="${run_dir}"
wait "${capture_pid}"
capture_pid=
cleanup
trap - EXIT

cp "${capture_file}" "${evidence_dir}/ros-certified-capture.json"
cp "${run_dir}/integrity.csv" "${evidence_dir}/ros-certified-integrity.csv"
sha256sum \
  "${evidence_dir}/capture_ros_publication.py" \
  "${evidence_dir}/reproduce_ros_certified_capture.sh" \
  /home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl/test/p005_ros_certified_publisher.cpp \
  "${capture_file}" \
  "${run_dir}/integrity.csv"
cat /tmp/p005_ros_certified_capture.log
