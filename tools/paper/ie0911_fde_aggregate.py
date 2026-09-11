#!/usr/bin/env python3
"""Aggregate frozen IE0911 residual-FDE runs without reading estimator truth."""

import argparse
import csv
import hashlib
import json
from pathlib import Path


FINAL_MODES = ("robust_cauchy", "suppress_all", "structured_debias",
               "lcb_partial", "lcb_fixed_full")
CACHE_FINAL_MODES = FINAL_MODES[1:]


def read_json(path):
    path = Path(path)
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}


def sha(path):
    path = Path(path)
    if not path.is_file():
        return None
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def available(document, key):
    value = document.get(key, {})
    return value.get("value") if value.get("status") == "AVAILABLE" else None


def csv_rows(path):
    path = Path(path)
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch", required=True)
    parser.add_argument("--evaluation", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    batch = read_json(args.batch)
    evaluation = read_json(args.evaluation)
    metrics = {row["cell_id"]: row for row in evaluation.get("run_metrics", [])}
    units = sorted({cell["run_unit_id"] for cell in batch["cells"]})
    records = []
    flat = []
    for unit in units:
        cells = [cell for cell in batch["cells"] if cell["run_unit_id"] == unit]
        producer = next(cell for cell in cells
                        if cell["execution_type"] == "CACHE_PRODUCER")
        producer_dir = Path(producer["run_directory"])
        fde = read_json(producer_dir / "fde_status.json")
        producer_status = read_json(producer_dir / "run_status.json")
        score_rows = csv_rows(producer_dir / "scores_decision.csv")
        rc_valid = sum(row.get("valid_score_exported", "").lower() in
                       {"1", "true"} for row in score_rows)
        final_records = {}
        for mode in FINAL_MODES:
            candidates = [cell for cell in cells
                          if cell["canonical_mode"] == mode and
                          cell["execution_type"] in {
                              "BASELINE_TRAJECTORY", "FINAL_TRAJECTORY"}]
            cell = candidates[0]
            metric_row = metrics.get(cell["cell_id"], {})
            trajectory = metric_row.get("trajectory", {})
            matched = available(trajectory, "matched_count")
            unmatched = available(trajectory, "unmatched_count")
            coverage = (matched / (matched + unmatched)
                        if matched is not None and unmatched is not None and
                        matched + unmatched else None)
            run_dir = Path(cell["run_directory"]) if cell.get("run_directory") else None
            run_status = read_json(run_dir / "run_status.json") if run_dir else {}
            compensations = (csv_rows(run_dir / "fixed_compensations.csv")
                             if run_dir else [])
            final = {
                "cell_status": cell["status"],
                "reason": cell.get("reason", run_status.get("reason")),
                "cache_id": cell.get("cache_id"),
                "final_status": run_status.get("status"),
                "fallback_attempted": bool(
                    run_status.get("fallback_attempt_count", 0)),
                "trajectory": {
                    "rmse_m": available(trajectory, "aligned_ATE_rmse_m"),
                    "p95_m": available(trajectory, "aligned_ATE_p95_m"),
                    "horizontal_rmse_m": available(
                        trajectory, "aligned_horizontal_rmse_m"),
                    "vertical_rmse_m": available(
                        trajectory, "aligned_height_rmse_m"),
                    "coverage": coverage,
                    "matched_gt_count": matched,
                },
                "fixed_compensation_count": len(compensations),
                "fixed_compensation_used_count": sum(
                    row.get("use", "").lower() in {"1", "true"}
                    for row in compensations),
                "paired_vs_suppress": metric_row.get(
                    "paired_vs_suppress", {}),
            }
            final_records[mode] = final
            flat.append({
                "run_unit_id": unit, "method": mode,
                "cell_status": final["cell_status"],
                "failure_reason": final["reason"],
                "cache_id": final["cache_id"],
                "fde_fault_count": fde.get("fault_count"),
                "fde_positive_candidate_count": fde.get(
                    "positive_candidate_count"),
                "fde_retained_segment_count": fde.get(
                    "retained_segment_count"),
                **final["trajectory"],
                "paired_status": final["paired_vs_suppress"].get("status"),
                "rmse_improvement_vs_suppress_m": final[
                    "paired_vs_suppress"].get(
                        "rmse_improvement_vs_suppress_m"),
                "rmse_improvement_vs_suppress_pct": final[
                    "paired_vs_suppress"].get(
                        "rmse_improvement_vs_suppress_pct"),
                "p95_improvement_vs_suppress_m": final[
                    "paired_vs_suppress"].get(
                        "p95_improvement_vs_suppress_m"),
                "horizontal_improvement_vs_suppress_m": final[
                    "paired_vs_suppress"].get(
                        "horizontal_rmse_improvement_vs_suppress_m"),
                "vertical_improvement_vs_suppress_m": final[
                    "paired_vs_suppress"].get(
                        "vertical_rmse_improvement_vs_suppress_m"),
                "coverage_change_pp_vs_suppress": final[
                    "paired_vs_suppress"].get(
                        "coverage_percentage_point_change_vs_suppress"),
                "coverage_relative_change_pct_vs_suppress": final[
                    "paired_vs_suppress"].get(
                        "coverage_relative_change_vs_suppress_pct"),
            })
        cache_ids = [final_records[mode]["cache_id"]
                     for mode in CACHE_FINAL_MODES]
        common_cache = (cache_ids[0] if cache_ids and
                        all(item == cache_ids[0] and item is not None
                            for item in cache_ids) else None)
        records.append({
            "run_unit_id": unit,
            "fde": fde or {"status": "UNAVAILABLE"},
            "fde_artifact_sha256": {
                name: sha(producer_dir / name) for name in
                ("fde_status.json", "fde_observations.csv",
                 "support_partition.json", "partition.json")},
            "stage2": {
                "cell_status": producer["status"],
                "solver_status": producer_status.get("segment_refit_status",
                                                      producer_status.get("solver_status")),
                "reason": producer.get("reason", producer_status.get("reason")),
                "cache_id": producer.get("cache_id"),
            },
            "recoverability_R_c": {
                "status": producer_status.get("recoverability_status",
                                              "NOT_RUN"),
                "group_count": len(score_rows),
                "valid_group_count": rc_valid,
            },
            "finals": final_records,
            "candidate_dependent_common_cache": {
                "status": "VERIFIED" if common_cache else "UNAVAILABLE",
                "cache_id": common_cache,
                "requested_cache_ids": dict(zip(CACHE_FINAL_MODES, cache_ids)),
            },
        })
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    result = {
        "schema": "uifgo_ie0911_fde_aggregate_v1",
        "batch_manifest_sha256": sha(args.batch),
        "evaluation_sha256": sha(args.evaluation),
        "improvement_sign": "suppress_all_minus_method",
        "percentage_denominator": "suppress_all",
        "records": records,
    }
    (output / "fde_aggregate.json").write_text(
        json.dumps(result, indent=2) + "\n", encoding="utf-8")
    with (output / "fde_aggregate.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
        writer.writeheader()
        writer.writerows(flat)
    print(json.dumps({"records": len(records), "rows": len(flat)}))


if __name__ == "__main__":
    main()
