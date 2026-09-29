#!/usr/bin/env python3
"""Compare P1-02 before/after outputs with explicit work-field exclusions."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import pathlib
import re


FILES = {
    "states.csv": {
        "numeric": {"px", "py", "pz", "qw", "qx", "qy", "qz", "vx",
                    "vy", "vz", "bax", "bay", "baz", "bgx", "bgy", "bgz"},
        "ignore": set(),
        "abs": 0.0,
        "rel": 0.0,
    },
    "integrity.csv": {
        "numeric": {"global_statistic", "global_threshold", "postfit_statistic",
                    "postfit_threshold", "conditional_statistic",
                    "conditional_threshold", "pl_x", "pl_y", "pl_z", "hpl_m",
                    "vpl_m", "allocated_hmi_risk", "hmi_risk_requirement",
                    "bridge_pl_x", "bridge_pl_y", "bridge_pl_z"},
        # The real-time performance run intentionally retains the actual wall
        # deadline.  Its packet digest and deadline bit therefore vary even
        # between two unchanged checkpoint repeats; they are summarized as
        # performance outcomes, not used as the deterministic computation
        # oracle.  Injected-clock deadline tests cover those boundary fields.
        "ignore": set(),
        "abs": 0.0,
        "rel": 0.0,
    },
    "diagnostic_candidates.csv": {
        "numeric": {"condition_lower_bound", "condition_upper_bound",
                    "certificate_margin"},
        "ignore": {"kernel_ms", "post_ms", "bridge_ms", "fault_map_ms",
                   "pl_ms", "cache_hits", "covariance_solve_count"},
        "abs": 0.0,
        "rel": 0.0,
    },
    "diagnostic_coverage.csv": {
        "numeric": set(), "ignore": set(), "abs": 0.0, "rel": 0.0,
    },
    "diagnostic_snapshot_identity.csv": {
        "numeric": set(), "ignore": set(), "abs": 0.0, "rel": 0.0,
    },
    "diagnostic_square_root.csv": {
        "numeric": {"r_diagonal_min", "r_diagonal_max", "condition_estimate",
                    "identity_residual_relative", "parity_relative_difference",
                    "solution_relative_difference", "forward_error_bound",
                    "statistic"},
        "ignore": set(), "abs": 0.0, "rel": 0.0,
    },
    "transactions.csv": {
        "numeric": set(), "ignore": set(), "abs": 0.0, "rel": 0.0,
    },
    "candidates.csv": {
        "numeric": set(),
        "ignore": {"evaluation_wall_ms"}, "abs": 0.0, "rel": 0.0,
    },
    "hypotheses.csv": {
        "numeric": set(), "ignore": set(), "abs": 0.0, "rel": 0.0,
    },
    "bridge.csv": {
        "numeric": set(), "ignore": set(), "abs": 0.0, "rel": 0.0,
    },
    "diagnostic_history_summary.csv": {
        "numeric": set(), "ignore": set(), "abs": 0.0, "rel": 0.0,
    },
    "diagnostic_state_steps.csv": {
        "numeric": set(), "ignore": set(), "abs": 0.0, "rel": 0.0,
    },
    "diagnostic_attempts.csv": {
        "numeric": set(),
        "ignore": {"base_svd", "base_llt", "base_state_solves",
                   "llt_state_solve_calls", "svd_state_solve_calls",
                   "detector_reference_qr", "candidate_reference_svd",
                   "candidate_inner_llt", "fault_gram_eigen",
                   "fault_gram_svd", "fault_gram_ldlt",
                   "low_dim_fault_gram", "generic_fault_gram_fallback",
                   "hypothesis_parallel_blocks", "hypothesis_shared_hits",
                   "hypothesis_shared_misses", "covariance_rhs_solves",
                   "covariance_rhs_columns", "spectral_rhs_solves",
                   "spectral_rhs_columns", "watchdog_wall_elapsed_ns",
                   "statistical_cache_hits", "statistical_cache_misses",
                   "cache_entries",
                   "watchdog_sensor_elapsed_ns", "watchdog_sensor_delta_ns",
                   "watchdog_sensor_lag_ns", "watchdog_sensor_stale",
                   "watchdog_reason"},
        "abs": 0.0,
        "rel": 0.0,
    },
}


def load(path: pathlib.Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        return list(reader.fieldnames or []), list(reader)


def same_number(left: str, right: str, absolute: float, relative: float) -> bool:
    try:
        a, b = float(left), float(right)
    except ValueError:
        return False
    if math.isnan(a) or math.isnan(b):
        return math.isnan(a) and math.isnan(b)
    if math.isinf(a) or math.isinf(b):
        return a == b
    return abs(a - b) <= absolute + relative * max(abs(a), abs(b))


def normalize_text(field: str, value: str) -> str:
    if field == "reason_codes":
        return ";".join(item for item in value.split(";")
                        if item != "FINISH_DEADLINE_MISSED")
    if field == "reason":
        match = re.match(
            r"FINISH_DEADLINE_MISSED after [0-9.eE+-]+ ms; upstream=(.*)",
            value)
        if match:
            return match.group(1)
        return value
    return value


def finish_deadline_present(row: dict[str, str]) -> bool:
    return ("FINISH_DEADLINE_MISSED" in row.get("reason_codes", "").split(";")
            and re.fullmatch(
                r"FINISH_DEADLINE_MISSED after [0-9.eE+-]+ ms; upstream=.*",
                row.get("reason", "")) is not None)


def proved_finish_wall_transition(left: dict[str, str],
                                  right: dict[str, str]) -> bool:
    """Accept only the exact terminal transformation performed by finish()."""
    left_finish = finish_deadline_present(left)
    right_finish = finish_deadline_present(right)
    if not left_finish and not right_finish:
        return False
    if normalize_text("reason", left.get("reason", "")) != normalize_text(
            "reason", right.get("reason", "")):
        return False
    if normalize_text("reason_codes", left.get("reason_codes", "")) != \
            normalize_text("reason_codes", right.get("reason_codes", "")):
        return False
    return ((left.get("deadline_missed") == "1") == left_finish and
            (right.get("deadline_missed") == "1") == right_finish)


def proved_publication_wall_bundle(left: dict[str, str],
                                   right: dict[str, str]) -> bool:
    def finish_side(row: dict[str, str]) -> bool:
        return (row.get("publication_refusal") == "FINISH_DEADLINE_MISSED" and
                row.get("publication_protected") == "0" and
                row.get("publication_unprotected") == "1" and
                row.get("watchdog_wall_timeout") == "1")
    def ordinary_side(row: dict[str, str]) -> bool:
        return (row.get("publication_refusal") != "FINISH_DEADLINE_MISSED" and
                row.get("watchdog_wall_timeout") == "0")
    return ((finish_side(left) and ordinary_side(right)) or
            (finish_side(right) and ordinary_side(left)))


def compare_file(before: pathlib.Path, after: pathlib.Path, policy: dict,
                 proved_wall_rows: set[int]) -> dict:
    before_fields, before_rows = load(before)
    after_fields, after_rows = load(after)
    mismatches = []
    if before_fields != after_fields:
        mismatches.append({"kind": "header", "before": before_fields,
                           "after": after_fields})
    if len(before_rows) != len(after_rows):
        mismatches.append({"kind": "row_count", "before": len(before_rows),
                           "after": len(after_rows)})
    for row_index, (left, right) in enumerate(zip(before_rows, after_rows), 1):
        for field in before_fields:
            if field in policy["ignore"]:
                continue
            if (before.name == "integrity.csv" and
                    field in {"deadline_missed", "reason_codes", "reason"} and
                    proved_finish_wall_transition(left, right)):
                continue
            if (before.name == "diagnostic_attempts.csv" and
                    row_index in proved_wall_rows and
                    field in {"publication_protected",
                              "publication_unprotected",
                              "publication_refusal",
                              "watchdog_wall_timeout"} and
                    proved_publication_wall_bundle(left, right)):
                continue
            equal = (same_number(left[field], right[field], policy["abs"],
                                 policy["rel"])
                     if field in policy["numeric"]
                     else normalize_text(field, left[field]) ==
                          normalize_text(field, right[field]))
            if not equal:
                mismatches.append({"kind": "value", "row": row_index,
                                   "field": field, "before": left[field],
                                   "after": right[field]})
                if len(mismatches) >= 50:
                    break
        if len(mismatches) >= 50:
            break
    return {
        "status": "PASS" if not mismatches else "FAIL",
        "rows": len(before_rows),
        "before_sha256": hashlib.sha256(before.read_bytes()).hexdigest(),
        "after_sha256": hashlib.sha256(after.read_bytes()).hexdigest(),
        "mismatches": mismatches,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("before", type=pathlib.Path)
    parser.add_argument("after", type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()
    _, before_integrity = load(args.before / "integrity.csv")
    _, after_integrity = load(args.after / "integrity.csv")
    proved_wall_rows = {
        index for index, (left, right) in enumerate(
            zip(before_integrity, after_integrity), 1)
        if proved_finish_wall_transition(left, right)
    }
    results = {name: compare_file(args.before / name, args.after / name, policy,
                                  proved_wall_rows)
               for name, policy in FILES.items()}
    payload = {
        "schema": "uwb-imu-pl/p1-02-output-equivalence/v1",
        "before": str(args.before),
        "after": str(args.after),
        "status": "PASS" if all(item["status"] == "PASS"
                                for item in results.values()) else "FAIL",
        "mechanically_proved_finish_wall_rows": sorted(proved_wall_rows),
        "policies": {name: {"numeric_fields": sorted(policy["numeric"]),
                             "ignored_performance_fields": sorted(policy["ignore"]),
                             "absolute_tolerance": policy["abs"],
                             "relative_tolerance": policy["rel"]}
                     for name, policy in FILES.items()},
        "files": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
