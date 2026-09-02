#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "${repository_root}/../.." && pwd)"
git_sha="$(git -C "${repository_root}" rev-parse HEAD)"
artifact_root="${1:-${repository_root}/results/stage1_baseline_${git_sha:0:12}}"
smoke_seconds="${STAGE1_SMOKE_SECONDS:-8}"
fixed_lag_epochs="${STAGE1_FIXED_LAG_EPOCHS:-20}"

if [[ -n "$(git -C "${repository_root}" status --porcelain)" ]]; then
  echo "stage-1 acceptance requires a clean Git worktree" >&2
  exit 2
fi
if [[ ! "${fixed_lag_epochs}" =~ ^[0-9]+$ ]] ||
   (( fixed_lag_epochs < 2 )); then
  echo "STAGE1_FIXED_LAG_EPOCHS must be at least 2" >&2
  exit 2
fi
if [[ ! "${smoke_seconds}" =~ ^[0-9]+$ ]] || (( smoke_seconds < 3 )); then
  echo "STAGE1_SMOKE_SECONDS must be at least 3" >&2
  exit 2
fi

if [[ -d "${artifact_root}" &&
      -n "$(find "${artifact_root}" -mindepth 1 -print -quit)" ]]; then
  echo "artifact directory already exists and is not empty: ${artifact_root}" >&2
  exit 2
fi
mkdir -p "${artifact_root}/build" "${artifact_root}/runs"
temporary_root="$(mktemp -d)"
trap 'rm -rf -- "${temporary_root}"' EXIT

source /opt/ros/noetic/setup.bash

run_build_and_tests() {
  local build_type="$1"
  local lower_type
  lower_type="$(printf '%s' "${build_type}" | tr '[:upper:]' '[:lower:]')"
  local detail_log="${temporary_root}/${lower_type}_build_and_test.log"
  local result_log="${temporary_root}/${lower_type}_test_results.log"
  local summary_path="${artifact_root}/build/${lower_type}_summary.txt"

  cd "${workspace_root}"
  catkin clean uwb_imu_pl -y >"${detail_log}" 2>&1
  catkin config --cmake-args "-DCMAKE_BUILD_TYPE=${build_type}" \
    >>"${detail_log}" 2>&1
  catkin build uwb_imu_pl --no-status >>"${detail_log}" 2>&1
  catkin run_tests uwb_imu_pl --no-status >>"${detail_log}" 2>&1
  catkin_test_results --verbose >"${result_log}" 2>&1

  {
    printf 'stage=1\nstatus=PASS\nbuild_type=%s\n' "${build_type}"
    printf 'git_sha=%s\ngit_dirty=false\n' "${git_sha}"
    printf 'commands=catkin clean uwb_imu_pl -y; catkin config --cmake-args -DCMAKE_BUILD_TYPE=%s; catkin build uwb_imu_pl --no-status; catkin run_tests uwb_imu_pl --no-status; catkin_test_results --verbose\n' "${build_type}"
    printf '\ncatkin_test_results:\n'
    cat "${result_log}"
  } >"${summary_path}"
}

run_build_and_tests Debug
run_build_and_tests Release

source "${workspace_root}/devel/setup.bash"

mode_test_binary="${workspace_root}/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_realtime_incremental"
config_test_binary="${workspace_root}/devel/.private/uwb_imu_pl/lib/uwb_imu_pl/test_integrity_config"
"${config_test_binary}" >"${artifact_root}/build/config_loading_tests.txt" 2>&1
"${mode_test_binary}" \
  --gtest_filter='UwbImuIncremental.RepeatedExecutionIsDeterministic:UwbImuIncremental.FixedLagRetainsThreeCompleteEpochsAndBoundedMetadata:UwbImuIncremental.FixedLagMatchesUnboundedBeforeAndAfterMarginalization' \
  >"${artifact_root}/build/mode_integration_tests.txt" 2>&1

run_ros_smoke() {
  local mode="$1"
  local lag="$2"
  local run_directory="${artifact_root}/runs/${mode}"
  local launch_log="${temporary_root}/${mode}_roslaunch.log"
  local launch_status=0

  set +e
  timeout --signal=INT --kill-after=15s "${smoke_seconds}s" \
    roslaunch uwb_imu_pl realtime_integrity_sim.launch \
      trajectory:=figure_eight fault_mode:=none random_seed:=20260901 \
      rviz:=false packet_loss_prob:=0.0 nlos_probability:=0.0 \
      fixed_lag_epochs:="${lag}" run_directory:="${run_directory}" \
      >"${launch_log}" 2>&1
  launch_status=$?
  set -e
  if [[ ${launch_status} -ne 0 && ${launch_status} -ne 124 &&
        ${launch_status} -ne 130 ]]; then
    cp "${launch_log}" "${artifact_root}/runs/${mode}_failed_roslaunch.log"
    echo "${mode} ROS smoke failed with status ${launch_status}" >&2
    return 1
  fi
  python3 "${repository_root}/tools/validate_run_schema.py" "${run_directory}"
}

run_ros_smoke full_history 0
run_ros_smoke fixed_lag "${fixed_lag_epochs}"

python3 - "${artifact_root}" "${git_sha}" "${fixed_lag_epochs}" <<'PY'
import csv
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
expected_sha = sys.argv[2]
fixed_lag = int(sys.argv[3])
results = {}
for mode, expected_lag in (("full_history", 0), ("fixed_lag", fixed_lag)):
    directory = root / "runs" / mode
    manifest = json.loads((directory / "run_manifest.json").read_text())
    summary = json.loads((directory / "summary.json").read_text())
    with (directory / "timing.csv").open(newline="") as stream:
        timing_rows = sum(1 for _ in csv.DictReader(stream))
    with (directory / "events.csv").open(newline="") as stream:
        events = list(csv.DictReader(stream))
    marginalizations = sum(
        row["event"] == "FIXED_LAG_MARGINALIZE" for row in events)
    checks = {
        "git_sha_matches": manifest["git_sha"] == expected_sha,
        "git_dirty_is_false": manifest["git_dirty"] is False,
        "fixed_lag_epochs_matches":
            manifest["fixed_lag_epochs"] == expected_lag,
        "processed_positive": summary["processed"] > 0,
        "errors_zero": summary["errors"] == 0,
        "timing_rows_positive": timing_rows > 0,
        "fixed_lag_marginalized":
            marginalizations > 0 if expected_lag else marginalizations == 0,
    }
    if not all(checks.values()):
        raise SystemExit(f"{mode} validation failed: {checks}")
    results[mode] = {
        "status": "PASS",
        "fixed_lag_epochs": expected_lag,
        "config_hash": manifest["config_hash"],
        "seed": manifest["seed"],
        "processed": summary["processed"],
        "committed": summary["committed"],
        "rejected": summary["rejected"],
        "errors": summary["errors"],
        "timing_rows": timing_rows,
        "marginalizations": marginalizations,
        "execution_command": manifest["execution_command"],
        "checks": checks,
    }

acceptance = {
    "stage": 1,
    "status": "PASS",
    "git_sha": expected_sha,
    "git_dirty": False,
    "builds": {"Debug": "PASS", "Release": "PASS"},
    "automatic_tests": "PASS",
    "runs": results,
}
(root / "acceptance_summary.json").write_text(
    json.dumps(acceptance, indent=2, sort_keys=True) + "\n")
PY

(
  cd "${artifact_root}"
  find . -type f ! -name checksums.sha256 -print0 |
    sort -z | xargs -0 sha256sum >checksums.sha256
  sha256sum --check checksums.sha256
)

if [[ -n "$(git -C "${repository_root}" status --porcelain)" ]]; then
  echo "worktree became dirty during acceptance" >&2
  exit 3
fi

printf 'PASS: stage-1 baseline %s\nartifacts: %s\n' \
  "${git_sha}" "${artifact_root}"
