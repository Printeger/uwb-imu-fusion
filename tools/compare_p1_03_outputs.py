#!/usr/bin/env python3
"""Strict P1-03 comparison with a separate carrier-contract allowlist."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import pathlib

import compare_p1_02_outputs as p102


# These are semantic changes explicitly authorized for P1-03.  They are kept
# separate from timing/work ignores and must all occur on the same attempt set.
AUTHORIZED_IDENTITY = {
    "diagnostic_attempts.csv": {"publication_history_summary_id"},
    "diagnostic_history_summary.csv": {"version_digest"},
    "diagnostic_snapshot_identity.csv": {"whitening_id", "identity_digest"},
}
AUTHORIZED_RANK = {
    "candidates.csv": {"dof", "threshold"},
    "diagnostic_history_summary.csv": {"emitted_rows", "nu_perp"},
    "diagnostic_square_root.csv": {"rows", "dof", "detector_only_rows"},
    "integrity.csv": {"conditional_dof", "conditional_threshold"},
}


def policies() -> dict:
    result = copy.deepcopy(p102.FILES)
    result["diagnostic_attempts.csv"]["numeric"].add("base_step_norm")
    result["diagnostic_attempts.csv"].update(abs=1e-12, rel=1e-10)
    result["diagnostic_history_summary.csv"]["numeric"].update(
        {"kappa_b", "constant_offset", "omega_trace", "xi_norm"})
    result["diagnostic_history_summary.csv"].update(abs=1e-12, rel=1e-10)
    result["diagnostic_state_steps.csv"]["numeric"].update(
        {"rotation_norm", "position_norm", "velocity_norm",
         "accel_bias_norm", "gyro_bias_norm", "epoch_norm"})
    result["diagnostic_state_steps.csv"].update(abs=1e-12, rel=1e-10)
    # ADR-0002's existing ill-conditioned square-root tier is 1e-7.  This is
    # an equivalence tolerance for diagnostic certificate residuals, not a
    # detector threshold or acceptance-budget change.
    result["diagnostic_square_root.csv"].update(abs=1e-7, rel=1e-8)
    result["integrity.csv"].update(abs=1e-12, rel=1e-10)
    return result


def load(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def changed_rows(before: pathlib.Path, after: pathlib.Path,
                 fields: set[str]) -> dict[str, list[int]]:
    left, right = load(before), load(after)
    return {field: [i for i, (a, b) in enumerate(zip(left, right), 1)
                    if a[field] != b[field]] for field in sorted(fields)}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("before", type=pathlib.Path)
    parser.add_argument("after", type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()

    policy = policies()
    _, before_integrity = p102.load(args.before / "integrity.csv")
    _, after_integrity = p102.load(args.after / "integrity.csv")
    wall_rows = {i for i, (left, right) in enumerate(
        zip(before_integrity, after_integrity), 1)
        if p102.proved_finish_wall_transition(left, right)}

    results = {}
    identity_deltas = {}
    rank_deltas = {}
    for name, item in policy.items():
        identity = AUTHORIZED_IDENTITY.get(name, set())
        rank = AUTHORIZED_RANK.get(name, set())
        item["ignore"].update(identity | rank)
        results[name] = p102.compare_file(
            args.before / name, args.after / name, item, wall_rows)
        if identity:
            identity_deltas[name] = changed_rows(
                args.before / name, args.after / name, identity)
        if rank:
            rank_deltas[name] = changed_rows(
                args.before / name, args.after / name, rank)

    # Every rank-derived field must change on exactly the same non-empty set.
    row_sets = [tuple(rows) for values in rank_deltas.values()
                for rows in values.values()]
    rank_closure_ok = bool(row_sets) and bool(row_sets[0]) and all(
        rows == row_sets[0] for rows in row_sets)

    # The corrected effective dimension can only remove certified directions.
    monotone_ok = True
    for name, fields in AUTHORIZED_RANK.items():
        left, right = load(args.before / name), load(args.after / name)
        for field in fields:
            if "threshold" not in field:
                for row in rank_deltas[name][field]:
                    monotone_ok &= float(right[row - 1][field]) < float(
                        left[row - 1][field])

    status = (all(item["status"] == "PASS" for item in results.values())
              and rank_closure_ok and monotone_ok)
    payload = {
        "schema": "uwb-imu-pl/p1-03-dual-reference-equivalence/v1",
        "before": str(args.before), "after": str(args.after),
        "status": "PASS" if status else "FAIL",
        "authorized_contract_deltas": {
            "identity_fields": identity_deltas,
            "rank_derived_fields": rank_deltas,
            "rank_delta_rows": list(row_sets[0]) if row_sets else [],
            "rank_closure_ok": rank_closure_ok,
            "rank_dimensions_strictly_decrease": monotone_ok,
            "threshold_formula_proof":
                "O12 independent corrected oracle; not ignored as performance",
        },
        "performance_ignores": {
            name: sorted(p102.FILES[name]["ignore"]) for name in policy},
        "files": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    return 0 if status else 1


if __name__ == "__main__":
    raise SystemExit(main())
