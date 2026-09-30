#!/usr/bin/env python3
"""P1-05 worker equivalence with exact algorithm-work accounting.

Only wall-clock diagnostics and the configured worker-use diagnostic may vary.
In particular, statistical cache hits/misses and every numerical work counter
are compared exactly; they are not performance-ignore fields.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import pathlib

import compare_p1_02_outputs as p102
import compare_p1_03_outputs as p103


WORKER_CONFIGURATION_DIAGNOSTICS = {"hypothesis_parallel_blocks"}
ATTEMPT_TIMING_DIAGNOSTICS = {
    "watchdog_wall_elapsed_ns",
    "watchdog_sensor_elapsed_ns",
    "watchdog_sensor_delta_ns",
    "watchdog_sensor_lag_ns",
    "watchdog_sensor_stale",
    "watchdog_reason",
}
CANDIDATE_TIMING_DIAGNOSTICS = {
    "kernel_ms", "post_ms", "bridge_ms", "fault_map_ms", "pl_ms",
}


def column_values(path: pathlib.Path, field: str) -> list[str]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [row[field] for row in csv.DictReader(stream)]


def policies() -> dict:
    policy = copy.deepcopy(p103.policies())
    # Candidate cache/solve counts are algorithm work, not timing.
    policy["diagnostic_candidates.csv"]["ignore"] = set(
        CANDIDATE_TIMING_DIAGNOSTICS)
    # All process and cache counters are exact.  The only worker-dependent
    # field describes the requested execution configuration, not algorithmic
    # work, so it is reported separately below.
    policy["diagnostic_attempts.csv"]["ignore"] = (
        ATTEMPT_TIMING_DIAGNOSTICS | WORKER_CONFIGURATION_DIAGNOSTICS)
    return policy


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("before", type=pathlib.Path)
    parser.add_argument("after", type=pathlib.Path)
    parser.add_argument("--before-workers", required=True, type=int)
    parser.add_argument("--after-workers", required=True, type=int)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()

    policy = policies()
    _, before_integrity = p102.load(args.before / "integrity.csv")
    _, after_integrity = p102.load(args.after / "integrity.csv")
    wall_rows = {i for i, (left, right) in enumerate(
        zip(before_integrity, after_integrity), 1)
        if p102.proved_finish_wall_transition(left, right)}
    results = {name: p102.compare_file(
        args.before / name, args.after / name, item, wall_rows)
        for name, item in policy.items()}
    status = all(item["status"] == "PASS" for item in results.values())
    parallel_before = column_values(
        args.before / "diagnostic_attempts.csv", "hypothesis_parallel_blocks")
    parallel_after = column_values(
        args.after / "diagnostic_attempts.csv", "hypothesis_parallel_blocks")
    payload = {
        "schema": "uwb-imu-pl/p1-05-worker-equivalence/v1",
        "before": str(args.before),
        "after": str(args.after),
        "before_workers": args.before_workers,
        "after_workers": args.after_workers,
        "status": "PASS" if status else "FAIL",
        "algorithm_work_contract": (
            "exact comparison, including statistical cache hits/misses, "
            "entries, candidate cache hits, solve counts, and all numerical "
            "work counters"),
        "worker_configuration_diagnostic": {
            "field": "hypothesis_parallel_blocks",
            "meaning": (
                "configured/used parallel blocks; expected to vary with the "
                "worker setting and never claimed as invariant algorithm work"),
            "before_values": parallel_before,
            "after_values": parallel_after,
        },
        "allowed_timing_diagnostics": {
            name: sorted(item["ignore"]) for name, item in policy.items()
            if item["ignore"]
        },
        "files": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    return 0 if status else 1


if __name__ == "__main__":
    raise SystemExit(main())
