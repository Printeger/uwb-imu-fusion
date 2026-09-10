#!/usr/bin/env python3
"""Aggregate frozen R08 cell evaluations and preserve failed matrix rows."""

import argparse
import csv
import datetime
import json
from pathlib import Path


BASES = ("a10_val_turn_01_seed20101", "a10_val_turn_02_seed20102")
SCENARIOS = ("los", "step1", "step2", "step3", "ramp_gentle", "ramp_steep")
POLICIES = ("suppress_all", "structured_debias", "fit_only", "s_fit", "full_gate")


def json_object(path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def csv_rows(path):
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))
    except FileNotFoundError:
        return []


def value(metric):
    return metric.get("value") if isinstance(metric, dict) else None


def status(metric):
    return metric.get("status") if isinstance(metric, dict) else "UNAVAILABLE"


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=Path, required=True)
    args = parser.parse_args()
    root = args.evidence_root.resolve()
    input_rows = []
    group_rows = []
    paired_rows = []
    comparable = []
    truth_times = []

    for base in BASES:
        for scenario in SCENARIOS:
            cell = f"{base}_{scenario}"
            attempt = root / "attempts" / cell
            output = attempt / "output"
            pipeline = json_object(output / "pipeline_status.json")
            run = json_object(attempt / "RUN_RESULT.json")
            ticket = json_object(attempt / "TICKET_CONSUMPTION.json")
            stage1 = json_object(output / "stage1/status.json") or {}
            stage2 = json_object(output / "stage2/status.json") or {}
            partition = csv_rows(output / "stage1/partition.csv")
            segments = csv_rows(output / "segments.csv")
            scores = csv_rows(output / "scores.csv")
            evaluation = json_object(root / "evaluation" / f"{cell}.json")
            if pipeline and pipeline.get("status") == "SCORED":
                matrix_status = "SCORED"
                reason = ""
            elif pipeline and pipeline.get("status"):
                matrix_status = "FAILED"
                reason = pipeline.get("reason", "")
            elif ticket:
                matrix_status = "INTERRUPTED"
                reason = "SYSTEM_CRASH_AFTER_TICKET_CONSUMPTION; EXECUTION_STAGE_NOT_RETRIED"
            else:
                matrix_status = "NOT_RUN"
                reason = "NO_TICKET"
            candidate_count = sum(int(row.get("obs_count", 0)) for row in partition)
            score_groups = len(scores)
            eligible = sum(row.get("eligible") in ("1", "true") for row in scores)
            unavailable = sum(row.get("status") != "OK" for row in scores)
            decision_vectors = {}
            for policy in ("fit_only", "s_fit", "full_gate"):
                decisions = csv_rows(output / "decisions" / f"{policy}.csv")
                decision_vectors[policy] = [row.get("decision") for row in decisions]
            actual_gate_identical = (matrix_status == "SCORED" and
                                     len({tuple(items) for items in decision_vectors.values()}) == 1)
            input_rows.append({
                "base": base,
                "scenario": scenario,
                "matrix_status": matrix_status,
                "failure_or_interrupt_reason": reason,
                "ticket_id": ticket.get("ticket_id", "") if ticket else "",
                "wrapper_exit_code": run.get("exit_code", "") if run else "",
                "wrapper_elapsed_s": run.get("elapsed_s", "") if run else "",
                "stage1_status": stage1.get("status", "UNAVAILABLE"),
                "stage1_reason": stage1.get("reason", ""),
                "stage1_outer": stage1.get("outers", ""),
                "stage1_elapsed_s": stage1.get("elapsed_seconds", ""),
                "stage2_status": stage2.get("status", "UNAVAILABLE"),
                "stage2_reason": stage2.get("reason", ""),
                "stage2_outer": stage2.get("outers", ""),
                "stage2_elapsed_s": stage2.get("elapsed_seconds", ""),
                "candidate_observations": candidate_count,
                "segments": len(partition),
                "short_segments": sum(row.get("short_support") in ("1", "true") for row in partition),
                "boundary_segments": sum(row.get("boundary") in ("1", "true") for row in segments),
                "groups": score_groups,
                "eligible_groups": eligible,
                "unavailable_groups": unavailable,
                "fit_only_decisions": ";".join(decision_vectors["fit_only"]),
                "s_fit_decisions": ";".join(decision_vectors["s_fit"]),
                "full_gate_decisions": ";".join(decision_vectors["full_gate"]),
                "three_gate_use_suppress_identical": int(actual_gate_identical),
                "independent_evaluation": "COMPLETE" if evaluation else "UNAVAILABLE",
            })
            for score in scores:
                full = {row["group_id"]: row for row in csv_rows(
                    output / "decisions/full_gate.csv")}.get(score["group_id"], {})
                group_rows.append({
                    "base": base,
                    "scenario": scenario,
                    "group_id": score["group_id"],
                    "ordinals": score["ordinals"],
                    "eligible": score["eligible"],
                    "score_status": score["status"],
                    "eta": score["eta"],
                    "s_m": score["s_m"],
                    "max_gamma": full.get("max_gamma", ""),
                    "eta_pass": full.get("eta_pass", ""),
                    "s_pass": full.get("s_pass", ""),
                    "gamma_pass": full.get("gamma_pass", ""),
                    "fit_only_decision": ({row["group_id"]: row for row in csv_rows(
                        output / "decisions/fit_only.csv")}.get(score["group_id"], {}).get("decision", "")),
                    "s_fit_decision": ({row["group_id"]: row for row in csv_rows(
                        output / "decisions/s_fit.csv")}.get(score["group_id"], {}).get("decision", "")),
                    "full_gate_decision": full.get("decision", ""),
                    "numerical_reason": score["numerical_reason"],
                    "linearization_id": score["linearization_id"],
                })
            if not evaluation:
                for policy in POLICIES:
                    paired_rows.append({
                        "base": base, "scenario": scenario, "policy": policy,
                        "result_status": "UNAVAILABLE", "final_status": "NOT_AVAILABLE",
                        "candidate_observations": candidate_count, "accepted_candidates": "",
                        "final_used_candidates": "", "full_rmse_m": "", "full_p95_m": "",
                        "full_matches": "", "historical_rmse_m": "", "historical_p95_m": "",
                        "historical_matches": "", "delta_rmse_vs_suppress_m": "",
                        "delta_p95_vs_suppress_m": "", "accepted_bias_rmse_status": "UNAVAILABLE",
                        "accepted_bias_rmse_m": "", "bad_correction_rate_status": "UNAVAILABLE",
                        "bad_correction_rate": "", "good_rejection_rate_status": "UNAVAILABLE",
                        "good_rejection_rate": "", "candidate_use_coverage": "",
                        "eligible_use_coverage": "", "overall_retained_fraction": "",
                        "fallback_attempted": "", "fallback_status": "", "final_elapsed_s": "",
                        "unavailable_reason": reason,
                    })
                continue
            truth_times.append(evaluation["truth_first_opened_after_freeze_utc"])
            for policy in POLICIES:
                result = evaluation["policies"][policy]
                full = result["trajectory_full"]
                historical = result["trajectory_historical_3_6_s"]
                delta = result["historical_delta_vs_suppress_all"]
                accepted_bias = result["decision_time"]["accepted_bias_rmse_m"]
                bad = result["decision_time"]["bad_correction_rate"]
                rejection = result["decision_time"]["good_correction_rejection_rate"]
                coverage = result["coverage"]
                fallback = result["fallback"]
                valid = result["status"].get("valid_estimate") is True
                paired_rows.append({
                    "base": base, "scenario": scenario, "policy": policy,
                    "result_status": "AVAILABLE" if valid else "UNAVAILABLE",
                    "final_status": result["status"].get("status", ""),
                    "candidate_observations": evaluation["candidate_observations"],
                    "accepted_candidates": result["accepted_candidate_observations"],
                    "final_used_candidates": result["final_used_candidate_observations"],
                    "full_rmse_m": value(full["rmse_m"]), "full_p95_m": value(full["p95_m"]),
                    "full_matches": full["matches"],
                    "historical_rmse_m": value(historical["rmse_m"]),
                    "historical_p95_m": value(historical["p95_m"]),
                    "historical_matches": historical["matches"],
                    "delta_rmse_vs_suppress_m": value(delta["rmse_m"]),
                    "delta_p95_vs_suppress_m": value(delta["p95_m"]),
                    "accepted_bias_rmse_status": status(accepted_bias),
                    "accepted_bias_rmse_m": value(accepted_bias),
                    "bad_correction_rate_status": status(bad),
                    "bad_correction_rate": value(bad),
                    "good_rejection_rate_status": status(rejection),
                    "good_rejection_rate": value(rejection),
                    "candidate_use_coverage": value(coverage["candidate_use_coverage"]),
                    "eligible_use_coverage": value(coverage["eligible_use_coverage"]),
                    "overall_retained_fraction": value(coverage["overall_retained_fraction"]),
                    "fallback_attempted": fallback.get("attempted"),
                    "fallback_status": fallback.get("status"),
                    "final_elapsed_s": result["elapsed_seconds"],
                    "unavailable_reason": "" if valid else result["status"].get("status", ""),
                })
                if policy == "structured_debias" and value(delta["rmse_m"]) is not None:
                    comparable.append({"base": base, "scenario": scenario,
                                       "rmse_delta_m": value(delta["rmse_m"]),
                                       "p95_delta_m": value(delta["p95_m"])})

    write_csv(root / "INPUT_RESULTS.csv", input_rows)
    write_csv(root / "GROUP_RESULTS.csv", group_rows)
    write_csv(root / "PAIRED_RESULTS.csv", paired_rows)
    summary = {
        "schema": "T10_A19_R08_LIMITED_VALIDATION_AGGREGATE_V1",
        "created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "matrix_cells": len(input_rows),
        "pipeline_scored_cells": sum(row["matrix_status"] == "SCORED" for row in input_rows),
        "failed_cells": sum(row["matrix_status"] == "FAILED" for row in input_rows),
        "interrupted_cells": sum(row["matrix_status"] == "INTERRUPTED" for row in input_rows),
        "independently_evaluated_cells": sum(row["independent_evaluation"] == "COMPLETE" for row in input_rows),
        "three_gate_decision_differences": [
            f"{row['base']}_{row['scenario']}" for row in input_rows
            if row["matrix_status"] == "SCORED" and not row["three_gate_use_suppress_identical"]],
        "structured_comparable_cells": len(comparable),
        "structured_historical_rmse_better_cells": sum(row["rmse_delta_m"] < 0 for row in comparable),
        "structured_historical_rmse_worse_cells": sum(row["rmse_delta_m"] > 0 for row in comparable),
        "structured_historical_p95_better_cells": sum(row["p95_delta_m"] < 0 for row in comparable),
        "structured_historical_p95_worse_cells": sum(row["p95_delta_m"] > 0 for row in comparable),
        "structured_deltas": comparable,
        "truth_first_opened_utc_min": min(truth_times) if truth_times else None,
        "interpretation": [
            "No gate policy produced a different Use/Suppress vector from another gate policy on any scored cell.",
            "The limited matrix does not show a repeated trajectory benefit from retaining structured corrections.",
            "This is two-base synthetic validation characterization, without gate lock, generalization, significance, or claim upgrade.",
        ],
    }
    (root / "EVALUATION_SUMMARY.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
