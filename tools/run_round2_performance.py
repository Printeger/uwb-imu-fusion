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

from gate_d_diagnostics import validate_attachments

from round2_common import FAIL, PASS, atomic_json, load_protocol

REPOSITORY = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE = REPOSITORY.parent.parent
EXECUTABLE = WORKSPACE / "devel/.private/uwb_imu_pl/lib/uwb_imu_pl/realtime_performance_benchmark"
CONFIG = REPOSITORY / "config/realtime_uwb_imu_pl_research.yaml"


def one_run(output: pathlib.Path, epochs: int, workers: int, cpu: str,
            expected_candidates: int) -> dict:
    environment = os.environ.copy()
    # Export belongs to a separate diagnostic run, never a timed runner child.
    environment.pop("UWB_IMU_PL_REPLAY_EXPORT_DIR", None)
    environment.pop("UWB_IMU_PL_REPLAY_ATTEMPTS", None)
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
    try:
        completed = subprocess.run(command, cwd=REPOSITORY, env=environment,
                                   text=True, capture_output=True)
    except OSError as error:
        result = summarize_run(output, epochs, workers, expected_candidates, -1)
        result["errors"].append(str(error))
        return result
    result = summarize_run(output, epochs, workers, expected_candidates,
                           completed.returncode)
    result.update(stdout=completed.stdout[-4000:], stderr=completed.stderr[-4000:])
    return result


def distribution(items):
    return {"count": len(items), "p50_ms": float(np.percentile(items, 50)) if items else None,
            "p95_ms": float(np.percentile(items, 95)) if items else None,
            "p99_ms": float(np.percentile(items, 99)) if items else None,
            "max_ms": max(items) if items else None}


def summarize_run(output, epochs, workers, expected_candidates, returncode=0):
    result = {"status": "INVALID", "workers": workers, "epochs": epochs,
              "returncode": returncode, "timing_complete": False, "timing_failures": 0,
              "core_total": distribution([]), "stages": {}, "errors": [],
              "candidate_count": {"expected": expected_candidates, "epochs": 0,
                                  "min": 0, "max": 0, "contract_met": False}}
    try:
        if returncode:
            raise ValueError(f"benchmark process failed: {returncode}")
        attempts, timing, candidates = validate_attachments(output, epochs, 100, expected_candidates)
        measured = range(101, epochs + 1)
        if epochs <= 100:
            raise ValueError("no measurement attempts")
        stages = {}
        counts = []
        numerical_paths, fallback_reasons, candidate_stages = {}, {}, {}
        recovered_downstream = {"post_passed": 0, "pl_evaluated": 0, "selected": 0}
        counters = {k: 0 for k in ("generated", "coverage_rejected", "kernel_evaluated",
                    "slow_path", "post_passed", "pl_evaluated", "selected", "cache_hits",
                    "recovered_replacement", "certificate_passed",
                    "matrix_free_step_rejected", "covariance_solve_count",
                    "scratch_reuse_count")}
        for attempt in measured:
            for name, row in timing[attempt].items():
                if row["status"] == "EXECUTED":
                    stages.setdefault(name, []).append(float(row["wall_ms"]))
            counts.append(len(candidates[attempt]))
            for row in candidates[attempt].values():
                counters["generated"] += 1
                path = row["numerical_path"] or "P0_LEGACY"
                numerical_paths[path] = numerical_paths.get(path, 0) + 1
                if row["fallback_reason"]:
                    fallback_reasons[row["fallback_reason"]] = fallback_reasons.get(row["fallback_reason"], 0) + 1
                for stage in ("kernel_ms", "post_ms", "bridge_ms", "fault_map_ms", "pl_ms"):
                    if row[stage]: candidate_stages.setdefault(stage, []).append(float(row[stage]))
                if row["recovered_replacement"] == "1":
                    for key in recovered_downstream: recovered_downstream[key] += int(row[key])
                for name in counters:
                    # Diagnostic v4 adds certificate/scratch/covariance counters.
                    # Keep the independent reader compatible with frozen v1-v3
                    # attachments, where those columns are intentionally absent.
                    if name != "generated": counters[name] += int(row.get(name) or 0)
        result.update(status=PASS, timing_complete=True,
                      core_total=distribution(stages["core_total"]),
                      stages={name: distribution(items) for name, items in stages.items()},
                      candidate_counters=counters, numerical_paths=numerical_paths,
                      reference_svd=counters["slow_path"],
                      fallback_reasons=fallback_reasons, recovered_downstream=recovered_downstream,
                      candidate_stages={name: distribution(items) for name, items in candidate_stages.items()},
                      candidate_count={"expected": expected_candidates, "epochs": len(counts),
                                       "min": min(counts), "max": max(counts), "contract_met": True})
    except (OSError, ValueError, KeyError, TypeError, csv.Error) as error:
        result["errors"].append(str(error))
        result["timing_failures"] += 1
    return result


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
    args.output.mkdir(parents=True, exist_ok=True)
    affinity = sorted(os.sched_getaffinity(0))[:4]
    affinity_valid = len(affinity) == 4
    cpu = ",".join(str(value) for value in affinity)
    runs = []
    for index in range(repeats):
        destination = args.output / f"workers4-repeat{index + 1}"
        if destination.exists():
            raise FileExistsError(f"preserving existing evidence: {destination}")
        if affinity_valid:
            runs.append(one_run(destination, epochs, 4, cpu, spec["candidate_count"]))
        else:
            run = summarize_run(destination, epochs, 4, spec["candidate_count"], -1)
            run["errors"].append("Gate D requires four available CPUs")
            runs.append(run)
    if args.profile == "full":
        ablation_path = args.output / "workers1-ablation"
        if ablation_path.exists():
            raise FileExistsError(f"preserving existing evidence: {ablation_path}")
        ablation = (one_run(ablation_path, epochs, 1, cpu, spec["candidate_count"]) if affinity_valid
                    else summarize_run(ablation_path, epochs, 1, spec["candidate_count"], -1))
    else:
        ablation = {"status": "NOT_RUN", "reason":
                    "single-worker ablation follows a passing 40 ms smoke"}
    valid = all(run["status"] == PASS for run in runs) and (
        args.profile != "full" or ablation["status"] == PASS)
    candidate_contract = all(run["candidate_count"]["contract_met"] for run in runs)
    worst_p99 = max((run["core_total"]["p99_ms"] or float("inf") for run in runs), default=float("inf"))
    status = PASS if valid and candidate_contract and worst_p99 <= spec["p99_limit_ms"] else (
        FAIL if valid else "INVALID")
    summary = {"schema_version": "uwb-imu-pl/round2-performance/v1",
               "status": status, "profile": args.profile,
               "protocol_sha256": digest, "cpu_affinity": cpu,
               "numeric_threads": 1, "workers": 4, "warmup_epochs": 100,
               "p99_limit_ms": spec["p99_limit_ms"], "worst_p99_ms": worst_p99 if np.isfinite(worst_p99) else None,
               "formal_gate_d_pass": status == PASS and args.profile == "full",
               "candidate_contract_met": candidate_contract,
               "peak_child_rss_kb": resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
               "runs": runs, "single_worker_ablation": ablation}
    atomic_json(args.output / "performance_summary.json", summary)
    return 0 if status == PASS else 1


if __name__ == "__main__":
    sys.exit(main())
