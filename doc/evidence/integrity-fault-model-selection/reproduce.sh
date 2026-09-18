#!/usr/bin/env bash
set -euo pipefail

repo="${1:-/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl}"
build="${2:-/tmp/uwb_imu_pl_fault_switch_repro/build}"
out="${3:-/tmp/uwb_imu_pl_fault_switch_repro/runs}"
mkdir -p "$out"

cmake -S "$repo" -B "$build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$build" --target r0_r1_development tests -j2

bin="$build/devel/lib/uwb_imu_pl/r0_r1_development"
lib="$build/devel/lib"
export LD_LIBRARY_PATH="$lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
export EIGEN_NUM_THREADS=1 UWB_IMU_PL_CANDIDATE_WORKERS=4

# Prepare S and SD configs from the checked-in S-only default.
cp "$repo/config/realtime_uwb_imu_pl_research.yaml" "$out/config_s.yaml"
cp "$out/config_s.yaml" "$out/config_sd.yaml"
sed -i 's/double_faults_enabled: false/double_faults_enabled: true/' \
  "$out/config_sd.yaml"

for spec in s1 sd1 s2 sd2 s3 sd3; do
  mode="${spec%%[0-9]*}"
  /usr/bin/time -f 'elapsed_s=%e\nmax_rss_kb=%M' -o "$out/$spec.resource" \
    taskset -c 0-3 "$bin" "$out/config_${mode}.yaml" "$out/$spec" \
    140 A_default_ramp "$repo/config/r0_r1_development_scenarios.yaml"
done

for scenario in C_uwb_fde D_imu_bridge E_union; do
  for mode in s sd; do
    taskset -c 0-3 "$bin" "$out/config_${mode}.yaml" \
      "$out/${scenario}_${mode}" 32 "$scenario" \
      "$repo/config/r0_r1_development_scenarios.yaml"
  done
done
