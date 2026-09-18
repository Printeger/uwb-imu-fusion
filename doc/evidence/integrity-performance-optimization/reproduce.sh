#!/usr/bin/env bash
set -euo pipefail

REPO=${REPO:-/home/mint/ws_fusion_uwb/src/uwb-imu-fusion-pl}
BUILD=${BUILD:-/home/mint/ws_fusion_uwb/build/uwb_imu_pl}
DEVEL=${DEVEL:-/home/mint/ws_fusion_uwb/devel/.private/uwb_imu_pl}
FROZEN=${FROZEN:-/tmp/uwb_imu_pl_perf_20260916}
CFG=${CFG:-$FROZEN/input/realtime_uwb_imu_pl_research.yaml}
SCENARIOS=${SCENARIOS:-$FROZEN/input/r0_r1_development_scenarios.yaml}
AFFINITY=${AFFINITY:-0,2,4,6}
MODE=${1:-help}

export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export MKL_NUM_THREADS=1
export EIGEN_DONT_PARALLELIZE=1
export UWB_IMU_PL_CANDIDATE_WORKERS=4

run_app() {
  local version=$1 output=$2 epochs=$3 scenario=$4
  local bin=$FROZEN/$version/bin/r0_r1_development
  local lib=$FROZEN/$version/lib
  mkdir -p "$output"
  env LD_LIBRARY_PATH="$lib:/usr/local/lib" \
    taskset -c "$AFFINITY" /usr/bin/time -v -o "$output.time" \
    "$bin" "$CFG" "$output" "$epochs" "$scenario" "$SCENARIOS"
}

case "$MODE" in
  tests)
    cmake --build "$BUILD" --target \
      test_integrity_v2 test_run_logger test_realtime_incremental \
      test_dense_oracle test_deterministic_event test_integrity_config \
      test_snapshot_integrity -j4
    test_dir=$DEVEL/lib/uwb_imu_pl
    for name in test_integrity_v2 test_run_logger test_realtime_incremental \
      test_dense_oracle test_deterministic_event test_integrity_config \
      test_snapshot_integrity; do
      "$test_dir/$name"
    done
    PYTHONPATH="$REPO/tools" python3 "$REPO/test/test_gate_d_tools.py"
    ;;
  main)
    root=${OUTPUT_ROOT:-$FROZEN/reproduce-main-$(date +%Y%m%d-%H%M%S)}
    for round in 1 2 3; do
      run_app baseline "$root/baseline_$round" 140 A_default_ramp
      run_app final "$root/final_$round" 140 A_default_ramp
    done
    ;;
  ablation)
    root=${OUTPUT_ROOT:-$FROZEN/reproduce-ablation-$(date +%Y%m%d-%H%M%S)}
    bin=$FROZEN/final/bin/r0_r1_development
    lib=$FROZEN/final/lib
    run_variant() {
      local label=$1; shift
      local output=$root/$label
      mkdir -p "$output"
      env LD_LIBRARY_PATH="$lib:/usr/local/lib" "$@" \
        taskset -c "$AFFINITY" /usr/bin/time -v -o "$output.time" \
        "$bin" "$CFG" "$output" 40 A_default_ramp "$SCENARIOS"
    }
    run_variant A UWB_IMU_PL_DISABLE_HYPOTHESIS_SHARED=1 \
      UWB_IMU_PL_DISABLE_HYPOTHESIS_BATCH=1 \
      UWB_IMU_PL_DISABLE_WINDOW_BDCSVD=1 UWB_IMU_PL_CANDIDATE_WORKERS=1
    run_variant B UWB_IMU_PL_DISABLE_HYPOTHESIS_BATCH=1 \
      UWB_IMU_PL_DISABLE_WINDOW_BDCSVD=1 UWB_IMU_PL_CANDIDATE_WORKERS=1
    run_variant C UWB_IMU_PL_DISABLE_WINDOW_BDCSVD=1 \
      UWB_IMU_PL_CANDIDATE_WORKERS=1
    run_variant D UWB_IMU_PL_DISABLE_WINDOW_BDCSVD=1 \
      UWB_IMU_PL_CANDIDATE_WORKERS=4
    run_variant E UWB_IMU_PL_CANDIDATE_WORKERS=4
    ;;
  fault)
    root=${OUTPUT_ROOT:-$FROZEN/reproduce-fault-$(date +%Y%m%d-%H%M%S)}
    for scenario in C_uwb_fde D_imu_bridge E_union; do
      run_app baseline "$root/baseline_$scenario" 32 "$scenario"
      run_app final "$root/final_$scenario" 32 "$scenario"
    done
    ;;
  mature)
    root=${OUTPUT_ROOT:-$FROZEN/reproduce-mature-$(date +%Y%m%d-%H%M%S)}
    run_app final "$root/final" 226 H_mature_union
    python3 "$REPO/tools/validate_run_schema.py" "$root/final"
    PYTHONPATH="$REPO/tools" python3 - "$root/final" <<'PY'
import pathlib
import sys
from gate_d_diagnostics import validate_attachments
validate_attachments(pathlib.Path(sys.argv[1]), 226,
                     expected_candidates=None, require_legacy_timing=False)
PY
    ;;
  *)
    echo "usage: $0 {tests|main|ablation|fault|mature}" >&2
    exit 2
    ;;
esac
