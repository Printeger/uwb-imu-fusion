#!/usr/bin/env python3
"""Run the preregistered Gate-D benchmark and summarize full-epoch latency."""

from __future__ import annotations

import argparse
import csv
import json
import os
import pathlib
import resource
import shutil
import subprocess
import sys

import numpy as np

from round2_common import FAIL, PASS, atomic_json, load_protocol

REPOSITORY = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE = REPOSITORY.parent.parent
EXECUTABLE = WORKSPACE / "devel/.private/uwb_imu_pl/lib/uwb_imu_pl/realtime_performance_benchmark"
CONFIG = REPOSITORY / "config/realtime_uwb_imu_pl_research.yaml"


def one_run(output: pathlib.Path, epochs: int, workers: int, cpu: str,
            expected_candidates: int) -> dict:
    environment = os.environ.copy()
    environment.update({name: "1" for name in
                        ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
                         "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS", "BLIS_NUM_THREADS")})
    environment["UWB_IMU_PL_CANDIDATE_WORKERS"] = str(workers)
    environment["UWB_IMU_PL_BENCHMARK_FORCE_ALARM"] = "1"
    library_paths = [WORKSPACE / "devel/.private/uwb_imu_pl/lib",
                     WORKSPACE / "devel/lib", pathlib.Path("/opt/ros/noetic/lib")]
    environment["LD_LIBRARY_PATH"] = ":".join(map(str, library_paths)) + (
        ":" + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else "")
    command = ["taskset", "-c", str(cpu), str(EXECUTABLE), str(CONFIG),
               str(output), str(epochs)]
    completed = subprocess.run(command, cwd=REPOSITORY, env=environment,
                               text=True, capture_output=True)
    if completed.returncode:
        return {"status": "INVALID", "returncode": completed.returncode,
                "stderr": completed.stderr[-4000:]}
    values, stages, timing_failures = [], {}, 0
    with (output / "timing.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row["cold_warm"] == "cold":
                continue
            value = float(row["wall_ms"])
            stages.setdefault(row["stage"], []).append(value)
            if row["success"] not in ("1", "true", "True"):
                timing_failures += 1
            if row["stage"] == "core_total":
                values.append(value)
    candidate_counts = {}
    with (output / "candidates.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            window = int(row["window_id"])
            candidate_counts[window] = candidate_counts.get(window, 0) + 1
    candidate_counts = {key: value for key, value in candidate_counts.items()
                        if value > 1}
    measured_candidate_counts = [candidate_counts[key]
                                 for key in sorted(candidate_counts)[-(epochs - 100):]]
    def summary(items):
        return {"count": len(items), "p50_ms": float(np.percentile(items, 50)),
                "p95_ms": float(np.percentile(items, 95)),
                "p99_ms": float(np.percentile(items, 99)), "max_ms": max(items)}
    timing_complete = len(values) == epochs - 100 and timing_failures == 0
    return {"status": PASS if timing_complete else "INVALID",
            "workers": workers, "epochs": epochs,
            "timing_complete": timing_complete,
            "timing_failures": timing_failures,
            "core_total": summary(values),
            "stages": {name: summary(items) for name, items in stages.items()},
            "candidate_count": {
                "expected": expected_candidates,
                "epochs": len(measured_candidate_counts),
                "min": min(measured_candidate_counts, default=0),
                "max": max(measured_candidate_counts, default=0),
                "contract_met": len(measured_candidate_counts) == epochs - 100 and
                    min(measured_candidate_counts, default=0) == expected_candidates},
            "stdout": completed.stdout[-4000:]}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("protocol", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--profile", choices=("full", "smoke"), default="full")
    args = parser.parse_args()
    protocol, digest = load_protocol(args.protocol)
    spec = protocol["gate_d"]
    repeats = spec["repeats"] if args.profile == "full" else 1
    epochs = spec["epochs_per_repeat"] if args.profile == "full" else 140
    if not EXECUTABLE.is_file():
        print(f"missing benchmark executable: {EXECUTABLE}", file=sys.stderr)
        return 2
    args.output.mkdir(parents=True, exist_ok=True)
    affinity = sorted(os.sched_getaffinity(0))[:4]
    if len(affinity) != 4:
        print("Gate D requires four CPUs in the available affinity mask", file=sys.stderr)
        return 2
    cpu = ",".join(str(value) for value in affinity)
    runs = []
    for index in range(repeats):
        destination = args.output / f"workers4-repeat{index + 1}"
        if destination.exists():
            shutil.rmtree(destination)
        runs.append(one_run(destination, epochs, 4, cpu, spec["candidate_count"]))
    ablation_path = args.output / "workers1-ablation"
    if ablation_path.exists():
        shutil.rmtree(ablation_path)
    ablation = one_run(ablation_path, epochs, 1, cpu, spec["candidate_count"])
    valid = all(run["status"] == PASS for run in runs) and ablation["status"] == PASS
    candidate_contract = all(run["candidate_count"]["contract_met"] for run in runs)
    worst_p99 = max((run["core_total"]["p99_ms"] for run in runs), default=float("inf"))
    status = PASS if valid and candidate_contract and worst_p99 <= spec["p99_limit_ms"] else (
        FAIL if valid else "INVALID")
    summary = {"schema_version": "uwb-imu-pl/round2-performance/v1",
               "status": status, "profile": args.profile,
               "protocol_sha256": digest, "cpu_affinity": cpu,
               "numeric_threads": 1, "workers": 4, "warmup_epochs": 100,
               "p99_limit_ms": spec["p99_limit_ms"], "worst_p99_ms": worst_p99,
               "candidate_contract_met": candidate_contract,
               "peak_child_rss_kb": resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
               "runs": runs, "single_worker_ablation": ablation}
    atomic_json(args.output / "performance_summary.json", summary)
    return 0 if status == PASS else 1


if __name__ == "__main__":
    sys.exit(main())
