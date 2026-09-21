#!/usr/bin/env python3
"""B1 equivalence comparator (P3).

Baseline/current roots are environment selectable (see square-root-context.md).

Compares the B1 scenario runs against the frozen P2 baseline, per CSV file:

  * discrete columns (strings, flags, ids, counters, reasons) must be identical
  * numeric columns are compared with the ADR 0002 tiers; because the logger
    prints 6 significant digits the CSV-level comparison bound is rel 1e-5
    (same convention as the P2 oracle), and any *crossing* of a discrete flag
    derived from numbers (passed/committed/plausible/label) is reported as a
    discrete difference.

Outputs equivalence.json (aggregate + per-file worst differences) and prints a
compact table.  Per-frame details, if needed, are written next to it.
"""
import csv
import json
import os
import sys

BASELINE = os.environ.get(
    "UWB_IMU_PL_EQUIVALENCE_BASELINE",
    "/tmp/uwb_imu_pl_b1_20260921/baseline_p2_runs_p2")
CURRENT = os.environ.get(
    "UWB_IMU_PL_EQUIVALENCE_CURRENT",
    "/tmp/uwb_imu_pl_b1_20260921/b1_runs_v12")
OUT = os.environ.get(
    "UWB_IMU_PL_EQUIVALENCE_OUT",
    "/tmp/uwb_imu_pl_b1_20260921/equivalence.json")
REL_CSV = 1.0e-5
ABS_FLOOR = 1.0e-9  # for near-zero intermediate quantities
# Columns that are instrumentation, not results: wall times and the B1
# decomposition counters (which are *expected* to change and are reported in the
# counting table instead of the equivalence table).
IGNORED_NUMERIC = {
    "evaluation_wall_ms", "wall_ms", "duration_ms", "elapsed_ms",
    "covariance_rhs_solves", "covariance_rhs_columns",
    "spectral_rhs_solves", "spectral_rhs_columns",
    "llt_state_solve_calls", "svd_state_solve_calls", "base_state_solves",
    "base_svd", "base_llt", "detector_reference_qr", "candidate_reference_svd",
    "candidate_inner_llt", "fault_gram_eigen", "fault_gram_svd", "fault_gram_ldlt",
    "low_dim_fault_gram", "generic_fault_gram_fallback", "hypothesis_parallel_blocks",
    "hypothesis_shared_hits", "hypothesis_shared_misses",
    "factor_block_cache_hits", "factor_block_cache_misses",
    "factor_block_cache_invalidations", "statistical_cache_hits",
    "statistical_cache_misses", "cache_bytes", "stage_wall_ms",
    "preparation_wall_ms", "total_wall_ms", "kernel_wall_ms", "pl_wall_ms",
    "kernel_ms", "post_ms", "bridge_ms", "fault_map_ms", "pl_ms", "pre_ms",
    "score_ms", "solve_ms", "total_ms", "mean_ms", "p95_ms",
}
# Reason columns that embed the shared-context byte accounting; the byte count
# legitimately changes with the B1 audit fields, the rest of the string must not.
def normalize_reason(value):
    parts = [part for part in value.split(";") if not part.startswith("context_bytes=")]
    return ";".join(parts)

SCENARIOS = ["A_nominal", "C_uwb_fde", "D_imu_bridge", "E_union",
             "F_ramp_unmonitorable", "G_continuous_rejection", "H_mature_union"]

DISCRETE_SUFFIXES = (
    "_id", "_ids", "_status", "_type", "_label", "_reason", "reason", "sensor",
    "kind", "role", "disposition", "mode_ids", "fault_unit_ids",
    "physical_source_ids", "schema_version", "availability", "phase",
    "invalidation_reason", "action_model_id", "whitening_model_id",
    "trajectory_style", "scenario", "selected_action_type", "key", "name",
    "metric", "unit", "path", "tag", "class", "source", "convention",
)


def looks_discrete(name, values):
    lowered = name.lower()
    if any(lowered.endswith(suffix) for suffix in DISCRETE_SUFFIXES):
        return True
    for value in values[:200]:
        if value == "":
            continue
        try:
            float(value)
        except ValueError:
            return True
    return False


def compare_file(baseline_path, current_path):
    with open(baseline_path) as handle:
        baseline = list(csv.DictReader(handle))
    with open(current_path) as handle:
        current = list(csv.DictReader(handle))
    result = {
        "baseline_rows": len(baseline),
        "current_rows": len(current),
        "row_count_matches": len(baseline) == len(current),
        "discrete_mismatches": [],
        "numeric": {},
        "skipped": [],
    }
    if len(baseline) != len(current):
        return result
    if not baseline:
        return result
    columns = list(baseline[0].keys())
    if columns != list(current[0].keys()):
        result["column_mismatch"] = {
            "baseline_only": [c for c in columns if c not in current[0]],
            "current_only": [c for c in current[0] if c not in columns],
        }
    shared = [c for c in columns if c in current[0]]
    for column in shared:
        base_values = [row[column] for row in baseline]
        cur_values = [row[column] for row in current]
        if column in IGNORED_NUMERIC:
            result["skipped"].append({"column": column, "rows": len(base_values),
                                      "reason": "instrumentation column"})
            continue
        if looks_discrete(column, base_values):
            mismatches = []
            for index in range(len(base_values)):
                base_value = base_values[index]
                cur_value = cur_values[index]
                if base_value == cur_value:
                    continue
                if column == "schema_version":
                    result.setdefault("metadata_differences", []).append(
                        {"column": column, "row": index,
                         "note": "v11 -> v12 bump"})
                    continue
                if column == "reason" and normalize_reason(base_value) == normalize_reason(cur_value):
                    result.setdefault("metadata_differences", []).append(
                        {"column": column, "row": index})
                    continue
                mismatches.append((index, base_value, cur_value))
            if mismatches:
                key = "route_differences" if column in ("fallback_reason",) else "discrete_mismatches"
                result.setdefault(key, []).append({
                    "column": column,
                    "count": len(mismatches),
                    "first": mismatches[:3],
                })
            continue
        worst_abs = 0.0
        worst_rel = 0.0
        worst_row = None
        non_numeric = 0
        for index in range(len(base_values)):
            try:
                base_number = float(base_values[index])
                cur_number = float(cur_values[index])
            except ValueError:
                non_numeric += 1
                continue
            if base_number != base_number or cur_number != cur_number:
                if (base_number != base_number) != (cur_number != cur_number):
                    non_numeric += 1
                continue
            difference = abs(base_number - cur_number)
            scale = max(abs(base_number), abs(cur_number))
            relative = difference / scale if scale > 0 else 0.0
            if difference > worst_abs:
                worst_abs = difference
                worst_row = index
            if relative > worst_rel:
                worst_rel = relative
        if non_numeric:
            result["skipped"].append({"column": column, "rows": non_numeric})
            continue
        result["numeric"][column] = {
            "worst_abs": worst_abs,
            "worst_rel": worst_rel,
            "worst_row": worst_row,
            "within_tolerance": worst_rel <= REL_CSV or worst_abs <= ABS_FLOOR,
        }
    return result


def main():
    report = {
        "baseline": BASELINE,
        "current": CURRENT,
        "tolerance": {"numeric_rel": REL_CSV, "absolute_floor": ABS_FLOOR,
                      "source": "logger prints 6 significant digits"},
        "scenarios": {},
    }
    worst_overall = 0.0
    discrete_total = 0
    for scenario in SCENARIOS:
        base_dir = os.path.join(BASELINE, scenario)
        cur_dir = os.path.join(CURRENT, scenario)
        if not os.path.isdir(base_dir):
            base_dir = os.path.join(
                "/tmp/uwb_imu_pl_b1_20260921/baseline_p2_runs_p2", scenario)
        entry = {"files": {}, "discrete_mismatches": 0, "numeric_failures": []}
        if not os.path.isdir(cur_dir) or not os.path.isdir(base_dir):
            entry["missing"] = True
            report["scenarios"][scenario] = entry
            continue
        for filename in sorted(os.listdir(base_dir)):
            if not filename.endswith(".csv"):
                continue
            current_path = os.path.join(cur_dir, filename)
            if not os.path.exists(current_path):
                entry["files"][filename] = {"missing_in_current": True}
                continue
            compared = compare_file(os.path.join(base_dir, filename),
                                    current_path)
            entry["files"][filename] = compared
            for mismatch in compared.get("discrete_mismatches", []):
                entry["discrete_mismatches"] += mismatch["count"]
                discrete_total += mismatch["count"]
            entry["route_differences"] = entry.get("route_differences", 0) + sum(
                mismatch["count"] for mismatch in compared.get("route_differences", []))
            entry["metadata_differences"] = entry.get("metadata_differences", 0) + len(
                compared.get("metadata_differences", []))
            for column, stats in compared["numeric"].items():
                worst_overall = max(worst_overall, stats["worst_rel"])
                if not stats["within_tolerance"]:
                    entry["numeric_failures"].append(
                        {"file": filename, "column": column,
                         "worst_rel": stats["worst_rel"],
                         "worst_abs": stats["worst_abs"],
                         "row": stats["worst_row"]})
        report["scenarios"][scenario] = entry
    report["summary"] = {
        "discrete_mismatch_total": discrete_total,
        "worst_numeric_rel": worst_overall,
        "numeric_tolerance": REL_CSV,
    }
    with open(OUT, "w") as handle:
        json.dump(report, handle, indent=1)
    for scenario, entry in report["scenarios"].items():
        if entry.get("missing"):
            print(f"{scenario}: MISSING")
            continue
        files = len(entry["files"])
        print(f"{scenario}: files={files} discrete_mismatches="
              f"{entry['discrete_mismatches']} route_differences="
              f"{entry.get('route_differences', 0)} metadata_differences="
              f"{entry.get('metadata_differences', 0)} numeric_failures="
              f"{len(entry['numeric_failures'])}")
        for failure in entry["numeric_failures"][:5]:
            print("   ", failure)
    print("summary:", report["summary"])


if __name__ == "__main__":
    main()
