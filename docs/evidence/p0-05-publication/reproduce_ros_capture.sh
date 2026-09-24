#!/usr/bin/env bash
set -euo pipefail

source /home/mint/ws_fusion_uwb/devel/setup.bash

evidence_dir=/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl/docs/evidence/p0-05-publication
run_dir=/tmp/p005_ros_repro
capture_file=/tmp/p005_ros_repro_capture.json
config=/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl/config/realtime_uwb_imu_pl_research.yaml

test ! -e "${run_dir}" || mv "${run_dir}" "${run_dir}.previous.$(date +%s)"

roslaunch uwb_imu_pl realtime_integrity_sim.launch \
  rviz:=false enable_run_logging:=true run_directory:="${run_dir}" \
  config_path:="${config}" > /tmp/p005_ros_repro_launch.log 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT "${launch_pid}" 2>/dev/null || true
  wait "${launch_pid}" 2>/dev/null || true
}
trap cleanup EXIT

python3 "${evidence_dir}/capture_ros_publication.py" \
  --require-committed --timeout 180 --output "${capture_file}"
cleanup
trap - EXIT

cp "${capture_file}" "${evidence_dir}/ros-capture-committed.json"
cp "${run_dir}/integrity.csv" "${evidence_dir}/ros-integrity-committed.csv"
sha256sum \
  "${evidence_dir}/capture_ros_publication.py" \
  "${evidence_dir}/reproduce_ros_capture.sh" \
  "${config}" \
  "${capture_file}" \
  "${run_dir}/integrity.csv" \
  "${run_dir}/run_manifest.json" \
  "${run_dir}/resolved_config.yaml"
