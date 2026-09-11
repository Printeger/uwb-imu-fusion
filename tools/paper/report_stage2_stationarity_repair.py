#!/usr/bin/env python3
"""Evaluate the locked Walk1 oracle run after the Stage2 stationarity repair."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import yaml

import nlos_injection_metrics as metrics


LOCKED_ROOT = Path("/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01")
REPOSITORY_HEAD = "3db4f3175fc647ac5a24e3de771e7e41ceddf4e7"
FINAL_CLASSIFICATION = "RECOVERY_BACKEND_OPERATIONAL_AFTER_CORRECTNESS_FIX"


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def digest(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def value(document: dict, key: str):
    return document.get(key, {}).get("value")


def compare(candidate: float, reference: float) -> dict:
    return {
        "absolute_rmse_change_m": candidate - reference,
        "relative_improvement_percent": (reference - candidate) / reference * 100.0,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--forensics", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    run = args.run.resolve()
    forensics = args.forensics.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)

    locked_path = LOCKED_ROOT / "locked_manifest.json"
    locked = load(locked_path)
    clean = next(item for item in locked["scenarios"]
                 if item["scenario_id"] == "sfuise_walk1_normal_clean")
    injected = next(item for item in locked["scenarios"]
                    if item["scenario_id"] == "sfuise_walk1_normal_injected")
    truth_path = Path(injected["truth"]) / "nlos_injection_truth.json"
    true_amplitude = float(load(truth_path)["specification"]["amplitude_m"])

    batch = load(run / "batch" / "batch_manifest.json")
    assert batch["producer_commit"] == REPOSITORY_HEAD
    cells = batch["cells"]
    producer = next(cell for cell in cells if cell["execution_type"] == "CACHE_PRODUCER")
    producer_run = Path(producer["run_directory"])
    final_cells = {cell["canonical_mode"]: cell for cell in cells
                   if cell["execution_type"] == "FINAL_TRAJECTORY"}
    baselines = {cell["canonical_mode"]: cell for cell in cells
                 if cell["execution_type"] == "BASELINE_TRAJECTORY"}

    support = load(run / "oracle_support.json")
    support_ids = set(support["obs_ids"])
    observations = {row["obs_id"]: row for row in rows(producer_run / "observations.csv")}
    actual = [observations[obs_id] for obs_id in support_ids if obs_id in observations]
    alignment = {
        "status": "PASS",
        "support_nonempty": bool(support_ids),
        "support_count": len(support_ids),
        "mapped_count": len(actual),
        "all_valid_and_planned": all(row["valid"] == "1" and row["planned"] == "1"
                                     for row in actual),
        "all_target_link": all(int(row["raw_tag_id"]) == support["tag_id"] and
                               int(row["anchor_id"]) == support["anchor_id"]
                               for row in actual),
        "unexpected_ids": sorted(support_ids - set(observations)),
        "actual_start_time": min(float(row["raw_time"]) for row in actual),
        "actual_end_time": max(float(row["raw_time"]) for row in actual),
        "support_source": support["support_source"],
    }
    if not all((alignment["support_nonempty"], alignment["all_valid_and_planned"],
                alignment["all_target_link"], not alignment["unexpected_ids"],
                alignment["support_count"] == 30)):
        raise AssertionError("locked oracle support alignment failed")

    support_yaml = yaml.safe_load((run / "oracle_support.yaml").read_text())
    effective = yaml.safe_load((producer_run / "batch_config_effective.yaml").read_text())
    forbidden_keys = {"amplitude_m", "amplitude", "bias_m", "c_true",
                      "ground_truth", "true_bias"}

    def forbidden(node) -> list[str]:
        found: list[str] = []
        if isinstance(node, dict):
            for key, child in node.items():
                if str(key).lower() in forbidden_keys:
                    found.append(str(key))
                found.extend(forbidden(child))
        elif isinstance(node, list):
            for child in node:
                found.extend(forbidden(child))
        return found

    trace_lines = (run / "estimator_file_access.trace").read_text().splitlines()
    opened_support = [line for line in trace_lines if "openat(" in line and
                      str(run / "oracle_support.yaml") in line]
    forbidden_opens = [line for line in trace_lines if "openat(" in line and any(
        marker in line for marker in
        ("nlos_injection_truth.json", "ground_truth.tum", "ground_truth.csv"))]
    truth_separation = {
        "status": "PASS",
        "support_forbidden_fields": forbidden(support_yaml),
        "effective_config_forbidden_fields": forbidden(effective),
        "support_open_count": len(opened_support),
        "forbidden_truth_or_gt_opens": forbidden_opens,
        "truth_tree_hidden_from_estimator": True,
        "true_amplitude_read_by": "POST_ESTIMATOR_EVALUATOR_ONLY",
    }
    if forbidden(support_yaml) or forbidden(effective) or forbidden_opens or not opened_support:
        raise AssertionError("truth separation failed")

    trace = rows(producer_run / "refit_iterations.csv")
    first, last = trace[0], trace[-1]
    segments = rows(producer_run / "segments.csv")
    scores = rows(producer_run / "scores_decision.csv")
    status = load(producer_run / "run_status.json")
    c_hat = float(segments[0]["amplitude_m"])
    sigma_c = float(scores[0]["s_m"])
    stage2 = {
        "solver_status": status["solver_status"],
        "termination": status["solver_termination"],
        "outer_iterations": len(trace),
        "conditional_solve_calls": len(trace),
        "fixed_checkpoint_restart_count": sum(
            int(row["conditional_fixed_checkpoint_restart_count"]) for row in trace),
        "checked_lm_optimizer_instances": len(trace) + sum(
            int(row["conditional_fixed_checkpoint_restart_count"]) for row in trace),
        "conditional_lm_iterate_calls_total": sum(
            int(row["conditional_lm_iterations"]) for row in trace),
        "conditional_lm_inner_iterations_total": sum(
            int(row["conditional_lm_inner_iterations"]) for row in trace),
        "initial_objective": float(first["objective_before"]),
        "final_objective": float(last["objective_after"]),
        "final_relative_objective_change": float(last["relative_objective_change"]),
        "final_scaled_state_step": float(last["scaled_state_step"]),
        "final_max_kkt_violation": float(last["max_kkt_violation"]),
        "final_max_scaled_navigation_gradient": float(
            last["max_scaled_navigation_gradient_objective"]),
        "navigation_stationarity_tolerance": float(
            last["navigation_stationarity_tolerance_objective"]),
        "navigation_gradient_roundoff_allowance": float(
            last["navigation_gradient_roundoff_allowance_objective"]),
        "final_predicates": {name: last[name] == "1" for name in
                             ("objective_ok", "step_ok", "kkt_ok",
                              "navigation_stationarity_ok")},
        "c_hat_m": c_hat,
        "c_gradient": float(segments[0]["gradient_objective_per_m"]),
        "inexact_handoff_used": False,
        "fixed_checkpoint_recovery_used": any(
            row["conditional_fixed_checkpoint_recovery_used"] == "1" for row in trace),
    }
    if status["solver_status"] != "CONVERGED" or not all(stage2["final_predicates"].values()):
        raise AssertionError("Stage2 did not satisfy the frozen joint contract")

    delta_lcb = max(0.0, c_hat - 2.0 * sigma_c)
    fixed = {}
    for method in ("lcb_fixed_full", "lcb_partial"):
        row = rows(Path(final_cells[method]["run_directory"]) / "fixed_compensations.csv")[0]
        fixed[method] = row
    if abs(float(fixed["lcb_fixed_full"]["delta_c_fixed_m"]) - c_hat) > 1e-14 or \
            abs(float(fixed["lcb_partial"]["delta_c_fixed_m"]) - delta_lcb) > 1e-14:
        raise AssertionError("fixed correction delivery mismatch")

    protocol = yaml.safe_load((Path(__file__).resolve().parents[2] /
        "config/paper/ie0911/step2_evaluation.yaml").read_text())["run_units"]["sfuise_walk1"]
    if digest(Path(protocol["ground_truth"])) != protocol["ground_truth_sha256"]:
        raise AssertionError("locked GT hash mismatch")

    localization = []
    clean_run = Path(clean["gate"]["methods"]["all_range"]["run_directory"])
    run_map = {
        "clean_raw": clean_run,
        "injected_raw": Path(baselines["all_range"]["run_directory"]),
        "robust_cauchy": Path(baselines["robust_cauchy"]["run_directory"]),
        **{method: Path(cell["run_directory"]) for method, cell in final_cells.items()},
    }
    for method, run_directory in run_map.items():
        measured = metrics.localization(run_directory, protocol, tuple(injected["window"]))
        localization.append({
            "method": method,
            "rmse_m": value(measured, "aligned_ATE_rmse_m"),
            "p95_m": value(measured, "aligned_ATE_p95_m"),
            "horizontal_rmse_m": value(measured, "aligned_horizontal_rmse_m"),
            "vertical_rmse_m": value(measured, "aligned_height_rmse_m"),
            "coverage": measured["trajectory_coverage"]["value"],
            "run_directory": str(run_directory),
        })
    with (output / "localization_metrics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(localization[0]))
        writer.writeheader()
        writer.writerows(localization)
    rmse = {row["method"]: row["rmse_m"] for row in localization}
    comparisons = {
        "lcb_vs_suppress": compare(rmse["lcb_partial"], rmse["suppress_all"]),
        "full_vs_suppress": compare(rmse["lcb_fixed_full"], rmse["suppress_all"]),
        "structured_vs_suppress": compare(rmse["structured_debias"], rmse["suppress_all"]),
        "lcb_vs_full": compare(rmse["lcb_partial"], rmse["lcb_fixed_full"]),
    }

    finals = {}
    for method, cell in final_cells.items():
        directory = Path(cell["run_directory"])
        summary = load(directory / "final_inference_summary.json")
        iterations = rows(directory / "final_refit_iterations.csv")
        audit = rows(directory / "final_factor_audit.csv")
        classifications: dict[str, int] = {}
        for row in audit:
            classifications[row["classification"]] = classifications.get(row["classification"], 0) + 1
        finals[method] = {
            "run_status": cell["attempts"][0]["run_status"],
            "valid_estimate": summary["valid_estimate"],
            "solver_status": summary["recovery_attempt_solver_status"],
            "optimizer_calls": len(iterations),
            "initial_objective": float(iterations[0]["objective_before"]),
            "final_objective": float(iterations[-1]["objective_after"]),
            "final_navigation_gradient": float(
                iterations[-1]["max_scaled_navigation_gradient_objective"]),
            "factor_count": summary["final_graph_factor_count"],
            "factor_classifications": classifications,
            "fallback_attempt_count": summary["fallback_attempt_count"],
            "covariance_status": summary["covariance_status"],
        }
        if not summary["valid_estimate"] or summary["recovery_attempt_solver_status"] != "CONVERGED":
            raise AssertionError(f"invalid final recovery for {method}")

    result = {
        "schema": "uifgo_stage2_stationarity_repair_result_v1",
        "baseline_head": REPOSITORY_HEAD,
        "locked_manifest_sha256": digest(locked_path),
        "truth_sha256": digest(truth_path),
        "batch_manifest_sha256": digest(run / "batch" / "batch_manifest.json"),
        "producer_binary_sha256": batch["producer_binary_sha256"],
        "producer_abi_sha256": batch["producer_abi_sha256"],
        "producer_toolchain_sha256": batch["producer_toolchain_sha256"],
        "oracle_support": support,
        "support_alignment": alignment,
        "truth_separation": truth_separation,
        "forensic_artifacts": {path.name: digest(path) for path in sorted(forensics.iterdir())
                               if path.is_file()},
        "stage2": stage2,
        "bias_evaluation": {
            "c_true_m": true_amplitude,
            "c_hat_m": c_hat,
            "signed_error_m": c_hat - true_amplitude,
            "absolute_error_m": abs(c_hat - true_amplitude),
        },
        "recoverability": {
            "Rc_status": scores[0]["status"],
            "rank": int(scores[0]["R_rank"]),
            "frozen_rank_certified": scores[0]["frozen_rank_certified"] == "1",
            "condition_1": float(scores[0]["condition_1"]),
            "rcond_1": float(scores[0]["rcond_1"]),
            "lambda_min_N_m2_inv": float(scores[0]["lambda_min_N_m2_inv"]),
            "sigma_c_m": sigma_c,
        },
        "corrections": {
            "kappa": 2.0,
            "delta_lcb_m": delta_lcb,
            "delta_full_m": c_hat,
            "lcb_over_correction": delta_lcb > true_amplitude,
            "full_over_correction": c_hat > true_amplitude,
            "lcb_residual_injected_bias_m": true_amplitude - delta_lcb,
        },
        "finals": finals,
        "localization": localization,
        "comparisons": comparisons,
        "scientific_scope": "BACKEND_ONLY_ORACLE_SUPPORT; NOT_END_TO_END_DETECTOR",
        "final_classification": FINAL_CLASSIFICATION,
        "localization_benefit": "LOCALIZATION_BENEFIT_OBSERVED"
            if rmse["lcb_partial"] < rmse["suppress_all"]
            else "NO_LOCALIZATION_BENEFIT_OBSERVED",
    }
    (output / "stage2_repair_result.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (output / "oracle_support_alignment_audit.json").write_text(
        json.dumps(alignment, indent=2) + "\n", encoding="utf-8")
    (output / "truth_separation_audit.json").write_text(
        json.dumps(truth_separation, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
