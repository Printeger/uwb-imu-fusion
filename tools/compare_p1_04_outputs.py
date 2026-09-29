#!/usr/bin/env python3
"""Strict P1-04 comparison; additive funnel rows live outside frozen files."""

from __future__ import annotations

import argparse
import json
import pathlib

import compare_p1_02_outputs as p102
import compare_p1_03_outputs as p103


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("before", type=pathlib.Path)
    parser.add_argument("after", type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()

    policy = p103.policies()
    _, before_integrity = p102.load(args.before / "integrity.csv")
    _, after_integrity = p102.load(args.after / "integrity.csv")
    wall_rows = {i for i, (left, right) in enumerate(
        zip(before_integrity, after_integrity), 1)
        if p102.proved_finish_wall_transition(left, right)}

    results = {name: p102.compare_file(
        args.before / name, args.after / name, item, wall_rows)
        for name, item in policy.items()}
    status = all(item["status"] == "PASS" for item in results.values())
    payload = {
        "schema": "uwb-imu-pl/p1-04-stream-equivalence/v1",
        "before": str(args.before),
        "after": str(args.after),
        "status": "PASS" if status else "FAIL",
        "allowed_semantic_delta": "none in frozen numerical/output files",
        "additive_diagnostic":
            "diagnostic_stages.csv candidate_audit reason funnel",
        "performance_ignores": {
            name: sorted(item["ignore"]) for name, item in policy.items()},
        "files": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    return 0 if status else 1


if __name__ == "__main__":
    raise SystemExit(main())
