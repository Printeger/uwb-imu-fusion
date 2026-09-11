#!/usr/bin/env python3
"""Create the frozen IE0911 STEP2 result tables and diagnostic plots."""
import argparse
import csv
import json
from pathlib import Path


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def value(document, key):
    item = document.get(key, {})
    return item.get("value") if item.get("status") == "AVAILABLE" else None


def csv_count(path, predicate):
    if not path.is_file():
        return None
    with path.open(newline="", encoding="utf-8") as stream:
        return sum(predicate(row) for row in csv.DictReader(stream))


def summarize(batch_path, evaluation_path, condition_kind):
    batch = read_json(batch_path)
    evaluation = read_json(evaluation_path)
    metric_by_cell = {row["cell_id"]: row for row in evaluation["run_metrics"]}
    producer_by_unit = {cell["run_unit_id"]: cell for cell in batch["cells"]
                        if cell["execution_type"] == "CACHE_PRODUCER"}
    rows = []
    for cell in batch["cells"]:
        if cell["execution_type"] not in {"BASELINE_TRAJECTORY", "FINAL_TRAJECTORY"}:
            continue
        metric = metric_by_cell[cell["cell_id"]]
        run_dir = Path(cell["run_directory"]) if cell.get("run_directory") else None
        run_status = (read_json(run_dir / "run_status.json") if run_dir and
                      (run_dir / "run_status.json").is_file() else {})
        producer = producer_by_unit.get(cell["run_unit_id"], {})
        producer_dir = Path(producer["run_directory"]) if producer.get("run_directory") else None
        producer_status = (read_json(producer_dir / "run_status.json")
                           if producer_dir and (producer_dir / "run_status.json").is_file()
                           else {})
        trajectory = metric["trajectory"]
        matched = value(trajectory, "matched_count")
        unmatched = value(trajectory, "unmatched_count")
        coverage = (matched / (matched + unmatched)
                    if matched is not None and unmatched is not None and
                    matched + unmatched else None)
        span = value(trajectory, "evaluation_interval_s")
        masks = run_dir / "final_masks.csv" if run_dir else Path("/nonexistent")
        candidate = csv_count(masks, lambda r: r.get("candidate", "").lower() in {"1", "true"})
        restored = csv_count(masks, lambda r: r.get("candidate", "").lower() in {"1", "true"}
                             and r.get("final_use", "").lower() in {"1", "true"})
        suppressed = (candidate - restored if candidate is not None and restored is not None
                      else None)
        failure_reason = cell.get("reason", "")
        failure_stage = run_status.get("failure_stage")
        if cell["status"] == "PARENT_CACHE_UNAVAILABLE":
            failure_stage = "PARENT_STAGE1_CACHE_UNAVAILABLE"
            failure_reason = (producer_status.get("discovery_status", "UNKNOWN") + ":" +
                              producer_status.get("reason", failure_reason))
        paired = metric.get("paired_vs_suppress", {})
        method = cell["canonical_mode"]
        if condition_kind == "main" and method == "lcb_fixed_full":
            condition = ("full_primary_full_compensation_ablation"
                         if cell["run_unit_id"] == "miluv_circular" else
                         "full_extra_full_compensation_diagnostic")
        else:
            condition = "full" if condition_kind == "main" else "low_redundancy_real_anchors"
        status = "OK" if cell["status"] == "COMPLETE" else "FAILED"
        effective = method if status == "OK" else "NONE"
        rows.append({
            "dataset": ("SFUISE" if cell["run_unit_id"].startswith("sfuise") else
                        "MILUV" if cell["run_unit_id"].startswith("miluv") else "OWN_VICON"),
            "sequence": cell["run_unit_id"], "condition": condition,
            "method": method,
            "code_config_identity": ";".join(filter(None, [
                batch.get("producer_binary_sha256"),
                (producer.get("common_preparation_id") if
                 cell.get("common_preparation_id") == "PENDING_ACTUAL_PREPARATION"
                 else cell.get("common_preparation_id")),
                cell.get("cache_id")])),
            "status": status, "failure_stage": failure_stage,
            "failure_reason": failure_reason,
            "fallback_used": bool(run_status.get("fallback_attempted", False) or
                                  run_status.get("status") == "FALLBACK_OK"),
            "effective_method": effective,
            "requested_time_span": "FULL_RECORDING_START_0_DURATION_MINUS_1",
            "evaluated_time_span": json.dumps(span) if span is not None else None,
            "coverage": coverage, "matched_gt_count": matched,
            "ate_frame_definition": "CONDITIONAL_SE3_ALIGNED_SCALE_1",
            "position_rmse_m": value(trajectory, "aligned_ATE_rmse_m"),
            "position_p95_m": value(trajectory, "aligned_ATE_p95_m"),
            "horizontal_rmse_m": value(trajectory, "aligned_horizontal_rmse_m"),
            "vertical_rmse_m": value(trajectory, "aligned_height_rmse_m"),
            "candidate_count": candidate, "restored_count": restored,
            "suppressed_count": suppressed,
            "elapsed_time_s": ((cell.get("attempts") or [{}])[-1].get("measured_wall_seconds")),
            "stage1_elapsed_s": (run_status.get("stage1_seconds") if run_dir else
                                 producer_status.get("stage1_seconds")),
            "stage2_elapsed_s": run_status.get("stage2_seconds"),
            "stage4_elapsed_s": run_status.get("stage4_engine_seconds"),
            "rmse_improvement_vs_suppress_m": paired.get("rmse_improvement_vs_suppress_m"),
            "rmse_improvement_vs_suppress_pct": paired.get("rmse_improvement_vs_suppress_pct"),
            "p95_improvement_vs_suppress_m": paired.get("p95_improvement_vs_suppress_m"),
            "paired_metric_status": paired.get("status", "UNAVAILABLE"),
            "anchor_ids": (json.dumps(cell.get("anchor_ids"))
                           if cell.get("anchor_ids") else "ALL_CONFIGURED_REAL"),
            "run_directory": str(run_dir) if run_dir else None,
            "producer_run_directory": str(producer_dir) if producer_dir else None,
        })
    return rows, list(producer_by_unit.values())


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--main-batch", required=True)
    parser.add_argument("--main-evaluation", required=True)
    parser.add_argument("--low-batch", required=True)
    parser.add_argument("--low-evaluation", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    output = Path(args.output)
    main_rows, main_producers = summarize(args.main_batch, args.main_evaluation, "main")
    low_rows, low_producers = summarize(args.low_batch, args.low_evaluation, "low")
    rows = main_rows + low_rows
    write_csv(output / "step2_results.csv", rows)
    range_rows = [{"dataset": r["dataset"], "sequence": r["sequence"],
                   "condition": r["condition"], "method": r["method"],
                   "status": "N/A", "sample_count": None,
                   "reason": "INDEPENDENT_ANCHOR_LEVER_TIME_STATIC_BETA_PROVENANCE_NOT_ESTABLISHED"}
                  for r in rows]
    write_csv(output / "range_metrics.csv", range_rows)

    trace_rows = []
    for producer in main_producers + low_producers:
        run_dir = Path(producer["run_directory"])
        status = read_json(run_dir / "run_status.json")
        trace_path = run_dir / "discovery_iterations.csv"
        traces = []
        if trace_path.is_file():
            with trace_path.open(newline="", encoding="utf-8") as stream:
                traces = list(csv.DictReader(stream))[-10:]
        if not traces:
            trace_rows.append({"run_unit_id": producer["run_unit_id"],
                "discovery_status": status.get("discovery_status"),
                "failure_reason": status.get("reason"), "outer_iteration": None,
                "objective_after": None, "relative_objective_change": None,
                "combined_scaled_step": None, "max_chain_kkt_objective_per_m": None,
                "navigation_gradient_objective": None, "objective_ok": None,
                "step_ok": None, "chain_optimality_ok": None,
                "navigation_stationarity_ok": None, "active_bias_count": None,
                "active_set_sha256": None, "active_set_symmetric_difference_count": None})
        for trace in traces:
            trace_rows.append({"run_unit_id": producer["run_unit_id"],
                "discovery_status": status.get("discovery_status"),
                "failure_reason": status.get("reason"),
                **{key: trace.get(key) for key in (
                    "outer_iteration", "objective_after", "relative_objective_change",
                    "combined_scaled_step", "max_chain_kkt_objective_per_m",
                    "navigation_gradient_objective", "objective_ok", "step_ok",
                    "chain_optimality_ok", "navigation_stationarity_ok",
                    "active_bias_count", "active_set_sha256",
                    "active_set_symmetric_difference_count")}})
    write_csv(output / "stage1_last10.csv", trace_rows)

    try:
        import matplotlib.pyplot as plt
        available = [r for r in rows if r["method"] == "robust_cauchy" and
                     r["position_rmse_m"] is not None]
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.bar([r["sequence"] for r in available],
               [r["position_rmse_m"] for r in available])
        ax.set_ylabel("Conditional SE(3)-aligned ATE RMSE (m)")
        ax.tick_params(axis="x", rotation=30)
        fig.tight_layout(); fig.savefig(output / "cauchy_aligned_ate.png", dpi=150)
        plt.close(fig)
        fig, ax = plt.subplots(figsize=(8, 4))
        for unit in sorted({r["run_unit_id"] for r in trace_rows}):
            selected = [r for r in trace_rows if r["run_unit_id"] == unit and
                        r["outer_iteration"] and r["active_bias_count"]]
            if selected:
                ax.plot([int(r["outer_iteration"]) for r in selected],
                        [int(r["active_bias_count"]) for r in selected],
                        marker="o", label=unit)
        ax.set_xlabel("Stage1 outer iteration")
        ax.set_ylabel("Active observation-bias count")
        if ax.lines:
            ax.legend()
        fig.tight_layout(); fig.savefig(output / "stage1_active_set_last10.png", dpi=150)
        plt.close(fig)
    except (ImportError, ValueError):
        pass
    summary = {
        "schema": "uifgo_ie0911_step2_aggregate_v1",
        "result_rows": len(rows), "main_result_rows": len(main_rows),
        "low_redundancy_result_rows": len(low_rows),
        "successful_trajectory_rows": sum(r["status"] == "OK" for r in rows),
        "available_suppress_pairs": sum(r["paired_metric_status"] == "AVAILABLE" for r in rows),
        "systemic_max_outer_blocker": sum(
            r["discovery_status"] == "MAX_OUTER_ITERATIONS" and
            r["run_unit_id"] in {p["run_unit_id"] for p in main_producers}
            for r in trace_rows if r["outer_iteration"] in {None, "50"}) >= 4,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n",
                                         encoding="utf-8")


if __name__ == "__main__":
    main()
