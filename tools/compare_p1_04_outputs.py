#!/usr/bin/env python3
"""Strict P1-04 comparison; additive funnel rows live outside frozen files."""

from __future__ import annotations

import argparse
import copy
import json
import pathlib

import compare_p1_02_outputs as p102
import compare_p1_03_outputs as p103


GATE24_AUDIT_EXACT_FIELDS = frozenset({
    "watchdog_sensor_elapsed_ns",
    "watchdog_sensor_delta_ns",
    "watchdog_sensor_lag_ns",
    "watchdog_sensor_stale",
    "watchdog_reason",
})


def compare_directories(before: pathlib.Path, after: pathlib.Path,
                        gate24_audit: bool = False) -> dict:
    """Run the established P1-04 quantity-aware golden comparator."""
    policy = p103.policies()
    if gate24_audit:
        # These are sensor-time health and terminal decision fields, not
        # performance counters.  Earlier P1 comparisons ignored them because
        # they predated the P1-06 audit contract.  Gate 2.4 overlays an exact
        # comparison without changing the established numerical policies.
        policy = copy.deepcopy(policy)
        policy["diagnostic_attempts.csv"]["ignore"].difference_update(
            GATE24_AUDIT_EXACT_FIELDS)
    _, before_integrity = p102.load(before / "integrity.csv")
    _, after_integrity = p102.load(after / "integrity.csv")
    wall_rows = {i for i, (left, right) in enumerate(
        zip(before_integrity, after_integrity), 1)
        if p102.proved_finish_wall_transition(left, right)}

    results = {name: p102.compare_file(
        before / name, after / name, item, wall_rows)
        for name, item in policy.items()}
    status = all(item["status"] == "PASS" for item in results.values())
    payload = {
        "schema": ("uwb-imu-pl/p1-06-gate24-audit-equivalence/v1"
                   if gate24_audit else
                   "uwb-imu-pl/p1-04-stream-equivalence/v1"),
        "before": str(before),
        "after": str(after),
        "status": "PASS" if status else "FAIL",
        "allowed_semantic_delta": "none in frozen numerical/output files",
        "additive_diagnostic":
            "diagnostic_stages.csv candidate_audit reason funnel",
        "performance_ignores": {
            name: sorted(item["ignore"]) for name, item in policy.items()},
        "files": results,
    }
    if gate24_audit:
        payload["audit_exact_fields"] = sorted(GATE24_AUDIT_EXACT_FIELDS)
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("before", type=pathlib.Path)
    parser.add_argument("after", type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument("--gate24-audit", action="store_true")
    args = parser.parse_args()

    payload = compare_directories(args.before, args.after, args.gate24_audit)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
