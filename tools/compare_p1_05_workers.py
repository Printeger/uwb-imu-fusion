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
import summarize_p1_06_simulation_acceptance as acceptance


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


def publication_bindings(directory: pathlib.Path) -> dict:
    """Use the existing per-attempt verifier, including packet/timing binding."""
    rows = {name: p103.load(directory/name) for name in (
        "diagnostic_attempts.csv", "integrity.csv", "transactions.csv",
        "terminal_packets.csv", "candidates.csv", "timing.csv")}
    arrival = [r for r in rows["timing.csv"] if r["stage"] == "arrival_to_publish"]
    count = len(rows["integrity.csv"])
    if any(len(rows[n]) != count for n in (
            "diagnostic_attempts.csv", "transactions.csv", "terminal_packets.csv")) or len(arrival) != count:
        return {"status": "FAIL", "failures": ["binding row count mismatch"]}
    failures = []
    for diag, integrity, transaction, terminal, timing in zip(
            rows["diagnostic_attempts.csv"], rows["integrity.csv"],
            rows["transactions.csv"], rows["terminal_packets.csv"], arrival):
        candidates = [r for r in rows["candidates.csv"]
                      if r["window_id"] == integrity["window_id"]]
        errors = acceptance.publication_failures(
            diag, integrity, candidates, integrity["deadline_missed"] == "1",
            True, transaction, terminal, float(timing["wall_ms"]))
        if errors:
            failures.append({"attempt": diag["input_attempt_id"], "errors": errors})
    return {"status": "FAIL" if failures else "PASS", "attempts": count,
            "failures": failures}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("before", type=pathlib.Path)
    parser.add_argument("after", type=pathlib.Path)
    parser.add_argument("--before-workers", required=True, type=int)
    parser.add_argument("--after-workers", required=True, type=int)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--performance-optimization", action="store_true",
                        help="Compare semantics and separately report changed work counters")
    args = parser.parse_args()

    policy = policies()
    work_deltas = {}
    if args.performance_optimization:
        work_fields = p102.FILES["diagnostic_attempts.csv"]["ignore"] - (
            ATTEMPT_TIMING_DIAGNOSTICS | WORKER_CONFIGURATION_DIAGNOSTICS)
        for name, fields in (("diagnostic_attempts.csv", work_fields),
                             ("diagnostic_candidates.csv", {"scratch_reuse_count"})):
            work_deltas[name] = p103.changed_rows(args.before/name, args.after/name, fields)
            policy[name]["ignore"].update(fields)
        policy["integrity.csv"]["wall_packet_digest"] = True
    _, before_integrity = p102.load(args.before / "integrity.csv")
    _, after_integrity = p102.load(args.after / "integrity.csv")
    wall_rows = {i for i, (left, right) in enumerate(
        zip(before_integrity, after_integrity), 1)
        if p102.proved_finish_wall_transition(left, right)}
    results = {name: p102.compare_file(
        args.before / name, args.after / name, item, wall_rows)
        for name, item in policy.items()}
    status = all(item["status"] == "PASS" for item in results.values())
    binding_checks = {}
    if args.performance_optimization:
        binding_checks = {"before": publication_bindings(args.before),
                          "after": publication_bindings(args.after)}
        status &= all(item["status"] == "PASS" for item in binding_checks.values())
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
            "changed work reported separately; all decision semantics compared" if args.performance_optimization else
            "exact comparison, including statistical cache hits/misses, "
            "entries, candidate cache hits, solve counts, and all numerical "
            "work counters"),
        "work_counter_changed_rows": work_deltas,
        "publication_binding_checks": binding_checks,
        "wall_packet_digest_contract": (
            "checksum may differ only on independently proved finish-wall rows; "
            "state/proof/publication binding fields are still compared"
            if args.performance_optimization else "exact"),
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
