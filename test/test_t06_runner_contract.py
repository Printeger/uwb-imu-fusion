#!/usr/bin/env python3
"""Actual short automatic run: no oracle/GT read, immutable partition, Stage 2."""

import argparse
import csv
import json
import math
import pathlib
import re
import subprocess
import tempfile


def load_strict_json(path: pathlib.Path) -> dict:
    def reject_constant(value):
        raise AssertionError(f"non-finite JSON token {value} in {path}")

    return json.loads(path.read_text("utf-8"), parse_constant=reject_constant)


def verify_stage1_diagnostic_row(row: dict) -> None:
    required = {
        "conditional_lm_inner_iterations",
        "conditional_lm_lambda",
        "conditional_lm_initial_error",
        "conditional_lm_check_status",
        "conditional_lm_check_evaluated",
        "conditional_lm_previous_error",
        "conditional_lm_current_error",
        "conditional_lm_relative_tolerance",
        "conditional_lm_absolute_tolerance",
        "conditional_lm_error_tolerance",
        "conditional_lm_absolute_decrease",
        "conditional_lm_relative_decrease",
        "conditional_lm_relative_decrease_valid",
        "conditional_lm_relative_tolerance_enabled",
        "conditional_lm_decrease_predicates_reached_by_linked_check",
        "conditional_lm_error_tolerance_triggered",
        "conditional_lm_absolute_tolerance_triggered",
        "conditional_lm_relative_tolerance_triggered",
        "conditional_lm_check_result",
        "conditional_lm_predicate_union_matches_check_result",
        "pre_chain_navigation_stationarity_valid",
        "pre_chain_navigation_stationarity_status",
        "pre_chain_max_scaled_gradient_objective",
        "post_chain_navigation_stationarity_valid",
        "post_chain_navigation_stationarity_status",
        "post_chain_max_scaled_gradient_objective",
        "stationarity_audits_share_navigation_values",
        "added_diagnostics_seconds",
        "conditional_lm_policy_version",
        "conditional_lm_stationarity_qualification_enabled",
        "conditional_lm_convergence_check_count",
        "conditional_lm_generic_convergence_count",
        "conditional_lm_qualification_evaluation_count",
        "conditional_lm_qualification_status",
        "conditional_lm_lambda_trial_accounting_status",
    }
    if not required.issubset(row):
        raise AssertionError(sorted(required - set(row)))
    previous = float(row["conditional_lm_previous_error"])
    current = float(row["conditional_lm_current_error"])
    absolute_tolerance = float(row["conditional_lm_absolute_tolerance"])
    relative_tolerance = float(row["conditional_lm_relative_tolerance"])
    error_tolerance = float(row["conditional_lm_error_tolerance"])
    absolute_decrease = previous - current
    relative_valid = previous != 0.0 and math.isfinite(
        absolute_decrease / previous)
    relative_decrease = absolute_decrease / previous \
        if relative_valid else None
    if not math.isclose(float(row["conditional_lm_absolute_decrease"]),
                        absolute_decrease, rel_tol=1e-15, abs_tol=1e-15):
        raise AssertionError(row)
    if (row["conditional_lm_relative_decrease_valid"] == "1") != \
            relative_valid:
        raise AssertionError(row)
    if relative_valid:
        if not math.isclose(float(row["conditional_lm_relative_decrease"]),
                            relative_decrease, rel_tol=1e-15, abs_tol=1e-15):
            raise AssertionError(row)
    elif row["conditional_lm_relative_decrease"] != "":
        raise AssertionError(row)
    expected = {
        "conditional_lm_error_tolerance_triggered":
            current <= error_tolerance,
        "conditional_lm_absolute_tolerance_triggered":
            absolute_decrease <= absolute_tolerance,
        "conditional_lm_relative_tolerance_triggered":
            relative_tolerance != 0.0 and relative_valid and
            relative_decrease <= relative_tolerance,
    }
    for field, value in expected.items():
        if (row[field] == "1") != value:
            raise AssertionError((field, value, row))
    reached_decrease = current > error_tolerance
    if (row["conditional_lm_decrease_predicates_reached_by_linked_check"] == "1") != reached_decrease:
        raise AssertionError(row)
    linked_result = expected["conditional_lm_error_tolerance_triggered"] or (
        reached_decrease and (
            expected["conditional_lm_absolute_tolerance_triggered"] or
            expected["conditional_lm_relative_tolerance_triggered"]))
    if (row["conditional_lm_check_result"] == "1") != linked_result:
        raise AssertionError(row)
    if row["conditional_lm_predicate_union_matches_check_result"] != "1":
        raise AssertionError(row)
    if row["conditional_lm_check_status"] != \
            "EVALUATED_MATCHED_LINKED_GTSAM":
        raise AssertionError(row)
    if row["conditional_lm_policy_version"] != "GTSAM_CHECK_ONLY_V1" or \
            row["conditional_lm_stationarity_qualification_enabled"] != "0" or \
            row["conditional_lm_qualification_evaluation_count"] != "0" or \
            row["conditional_lm_qualification_status"] != "NOT_ENABLED" or \
            row["conditional_lm_lambda_trial_accounting_status"] != \
            "COMPLETE":
        raise AssertionError(row)
    if row["pre_chain_navigation_stationarity_valid"] != "1" or \
            row["post_chain_navigation_stationarity_valid"] != "1" or \
            row["stationarity_audits_share_navigation_values"] != "1":
        raise AssertionError(row)
    if float(row["post_chain_max_scaled_gradient_objective"]) != \
            float(row["navigation_gradient_objective"]):
        raise AssertionError(row)
    if float(row["added_diagnostics_seconds"]) < 0.0:
        raise AssertionError(row)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runner", type=pathlib.Path, required=True)
    parser.add_argument("--source-config", type=pathlib.Path, required=True)
    parser.add_argument("--bag", type=pathlib.Path, required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="uifgo-t06-runner-test-") as temp:
        root = pathlib.Path(temp)
        config = root / "automatic.yaml"
        text = args.source_config.read_text(encoding="utf-8")
        text, count = re.subn(
            r"(?m)^  path:.*sim_circle_2026-06-15-16-04-35\.bag$",
            f"  path: {args.bag.resolve()}", text)
        if count != 1 or "oracle_support" in text:
            raise AssertionError("automatic config must contain one bag and no oracle input")
        config.write_text(text, encoding="utf-8")

        invalid_cases = {
            "score_disabled": re.sub(
                r"(?m)^  score_recoverability: true$",
                "  score_recoverability: false", text),
            "missing_lambda": re.sub(
                r"(?m)^  lambda_l1:.*\n", "", text),
            "empty_oracle": re.sub(
                r"(?m)^  score_recoverability: true$",
                "  score_recoverability: true\n  oracle_support: ''", text),
            "invalid_conditional_policy": re.sub(
                r"(?m)^  score_recoverability: true$",
                "  score_recoverability: true\n"
                "  discovery_conditional_navigation_policy: UNKNOWN_V9",
                text),
        }
        for run_id, invalid_text in invalid_cases.items():
            invalid_config = root / f"{run_id}.yaml"
            invalid_config.write_text(invalid_text, encoding="utf-8")
            invalid = subprocess.run(
                [str(args.runner.resolve()), "--config", str(invalid_config),
                 "--output-root", str(root), "--run-id", run_id],
                text=True, capture_output=True, check=False)
            if invalid.returncode == 0:
                raise AssertionError((run_id, invalid.stdout, invalid.stderr))
            if (root / run_id).exists():
                raise AssertionError(f"{run_id} passed config preflight far enough to create a run")

        stage1_failure_text, count = re.subn(
            r"(?m)^  admm_max_iterations:.*$",
            "  admm_max_iterations: 1", text)
        if count != 1:
            raise AssertionError("could not override admm_max_iterations")
        stage1_failure_config = root / "stage1_failure.yaml"
        stage1_failure_config.write_text(stage1_failure_text, encoding="utf-8")
        stage1_failure = subprocess.run(
            [str(args.runner.resolve()), "--config", str(stage1_failure_config),
             "--output-root", str(root), "--run-id", "stage1_failure"],
            text=True, capture_output=True, check=False)
        stage1_failure_run = root / "stage1_failure"
        if stage1_failure.returncode == 0:
            raise AssertionError(stage1_failure.stdout)
        stage1_status = json.loads(
            (stage1_failure_run / "run_status.json").read_text("utf-8"))
        if stage1_status.get("failure_stage") != "AUTOMATIC_DISCOVERY":
            raise AssertionError(stage1_status)
        for artifact in ("input_manifest.json", "config_original.yaml",
                         "config_effective.yaml", "capability_status.json",
                         "discovery_failure_diagnostics.json",
                         "support_snapshot.csv", "discovery_iterations.csv",
                         "admm_trace.csv", "partition.json"):
            if not (stage1_failure_run / artifact).exists():
                raise AssertionError(f"Stage-1 failure omitted {artifact}")
        failure_diagnostics = json.loads(
            (stage1_failure_run / "discovery_failure_diagnostics.json")
            .read_text("utf-8"))
        if failure_diagnostics.get("diagnostic_schema") != \
                "t10_stage1_diagnostics_v2":
            raise AssertionError(failure_diagnostics)
        last_lm = failure_diagnostics.get("last_conditional_lm")
        if not isinstance(last_lm, dict) or \
                last_lm.get("check_evaluated") is not True or \
                last_lm.get("predicate_union_matches_check_result") is not True:
            raise AssertionError(failure_diagnostics)
        if last_lm.get("lambda_trial_accounting_status") != "COMPLETE":
            raise AssertionError(failure_diagnostics)
        if last_lm.get("policy_version") != "GTSAM_CHECK_ONLY_V1" or \
                last_lm.get("stationarity_qualification_enabled") is not False or \
                last_lm.get("qualification_evaluation_count") != 0 or \
                last_lm.get("qualification_status") != "NOT_ENABLED":
            raise AssertionError(failure_diagnostics)
        if (stage1_failure_run / "trajectory.tum").exists():
            raise AssertionError("Stage-1 failure exported a trajectory")

        # A02 qualification must preserve its last stationarity audit when the
        # original total conditional LM budget is exhausted. This is a
        # targeted test run in a temporary directory, not a paper run.
        qualification_cap_text, count = re.subn(
            r"(?m)^  lm_max_iter:.*$", "  lm_max_iter: 1", text)
        if count != 1:
            raise AssertionError("could not override lm_max_iter")
        qualification_cap_text, count = re.subn(
            r"(?m)^  rel_error_tol:.*$", "  rel_error_tol: 1.0",
            qualification_cap_text)
        if count != 1:
            raise AssertionError("could not override rel_error_tol")
        qualification_cap_text, count = re.subn(
            r"(?m)^  navigation_stationarity_tolerance_objective:.*$",
            "  navigation_stationarity_tolerance_objective: 0.0",
            qualification_cap_text)
        if count != 1:
            raise AssertionError("could not override stationarity tolerance")
        qualification_cap_text, count = re.subn(
            r"(?m)^  score_recoverability: true$",
            "  score_recoverability: true\n"
            "  discovery_conditional_navigation_policy: "
            "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1",
            qualification_cap_text)
        if count != 1:
            raise AssertionError("could not enable A02 policy")
        qualification_cap_config = root / "qualification_cap.yaml"
        qualification_cap_config.write_text(
            qualification_cap_text, encoding="utf-8")
        qualification_cap = subprocess.run(
            [str(args.runner.resolve()), "--config",
             str(qualification_cap_config), "--output-root", str(root),
             "--run-id", "qualification_cap"],
            text=True, capture_output=True, check=False)
        qualification_cap_run = root / "qualification_cap"
        if qualification_cap.returncode == 0:
            raise AssertionError(qualification_cap.stdout)
        qualification_status = load_strict_json(
            qualification_cap_run / "run_status.json")
        if qualification_status.get("discovery_status") != \
                "CONDITIONAL_LM_FAILED":
            raise AssertionError(qualification_status)
        qualification_failure = load_strict_json(
            qualification_cap_run / "discovery_failure_diagnostics.json")
        qualification_lm = qualification_failure.get("last_conditional_lm")
        if not isinstance(qualification_lm, dict) or \
                qualification_lm.get("reason") != \
                "CONDITIONAL_LM_STATIONARITY_NOT_REACHED" or \
                qualification_lm.get("qualification_status") != \
                "NOT_STATIONARY_BUDGET_EXHAUSTED" or \
                qualification_lm.get("qualification_last_evaluated") is not True:
            raise AssertionError(qualification_failure)
        if qualification_lm.get("lambda_trial_accounting_status") != \
                "COMPLETE":
            raise AssertionError(qualification_failure)
        qualification_audit = qualification_lm.get(
            "last_qualification_stationarity")
        if not isinstance(qualification_audit, dict) or \
                qualification_audit.get("valid") is not True or \
                qualification_audit.get("stationary") is not False:
            raise AssertionError(qualification_failure)
        if qualification_lm.get(
                "optimizer_internal_relative_tolerance") != 1.0 or \
                qualification_lm.get(
                    "optimizer_internal_small_change_stop_enabled") is not True:
            raise AssertionError(qualification_failure)
        if (qualification_cap_run / "trajectory.tum").exists():
            raise AssertionError("qualification cap exported a trajectory")

        # A05 V2 changes only the optimizer-internal small-change search
        # tolerance. The external generic check and the AND stationarity
        # qualification remain the configured values, with a distinct solver
        # and producer identity.
        qualification_v2_text = qualification_cap_text.replace(
            "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1",
            "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2")
        qualification_v2_config = root / "qualification_cap_v2.yaml"
        qualification_v2_config.write_text(
            qualification_v2_text, encoding="utf-8")
        qualification_v2 = subprocess.run(
            [str(args.runner.resolve()), "--config",
             str(qualification_v2_config), "--output-root", str(root),
             "--run-id", "qualification_cap_v2"],
            text=True, capture_output=True, check=False)
        qualification_v2_run = root / "qualification_cap_v2"
        if qualification_v2.returncode == 0:
            raise AssertionError(qualification_v2.stdout)
        qualification_v2_failure = load_strict_json(
            qualification_v2_run / "discovery_failure_diagnostics.json")
        qualification_v2_lm = qualification_v2_failure.get(
            "last_conditional_lm")
        if not isinstance(qualification_v2_lm, dict) or \
                qualification_v2_lm.get("policy_version") != \
                "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2" or \
                qualification_v2_lm.get(
                    "optimizer_internal_relative_tolerance") != 0.0 or \
                qualification_v2_lm.get(
                    "optimizer_internal_small_change_stop_enabled") is not False or \
                qualification_v2_lm.get("relative_tolerance") != 1.0 or \
                qualification_v2_lm.get("absolute_tolerance") != 1e-8 or \
                qualification_v2_lm.get("stationarity_qualification_enabled") is not True or \
                qualification_v2_lm.get("lambda_trial_accounting_status") != \
                "COMPLETE":
            raise AssertionError(qualification_v2_failure)
        v1_manifest = load_strict_json(
            qualification_cap_run / "input_manifest.json")
        v2_manifest = load_strict_json(
            qualification_v2_run / "input_manifest.json")
        if v1_manifest.get("solver_config_hash_sha256") == \
                v2_manifest.get("solver_config_hash_sha256"):
            raise AssertionError("V1 and V2 solver identities collided")
        v2_effective = (qualification_v2_run / "config_effective.yaml").read_text(
            encoding="utf-8")
        for line in (
                "discovery_conditional_lm_optimizer_internal_relative_tolerance: 0",
                "discovery_conditional_lm_external_relative_tolerance: 1",
                "discovery_conditional_lm_external_absolute_tolerance: 1e-08"):
            if line not in v2_effective:
                raise AssertionError((line, v2_effective))
        if (qualification_v2_run / "trajectory.tum").exists():
            raise AssertionError("V2 qualification cap exported a trajectory")

        completed = subprocess.run(
            [str(args.runner.resolve()), "--config", str(config),
             "--output-root", str(root), "--run-id", "automatic"],
            text=True, capture_output=True, check=False)
        run = root / "automatic"
        status = json.loads((run / "run_status.json").read_text("utf-8"))
        if completed.returncode not in (0, 1):
            raise AssertionError(completed.stderr)
        if status["status"] not in (
                "OK", "NO_CANDIDATES", "PARTIAL_STAGE2_OK_SCORE_UNAVAILABLE"):
            raise AssertionError(status)
        manifest = json.loads((run / "input_manifest.json").read_text("utf-8"))
        if manifest.get("gt_read") is not False:
            raise AssertionError(manifest)
        if manifest.get("oracle_support_read") is not False:
            raise AssertionError(manifest)
        if manifest.get("gt_or_oracle_read") is not False:
            raise AssertionError(manifest)
        if (run / "oracle_support.yaml").exists():
            raise AssertionError("automatic run copied an oracle manifest")
        partition = json.loads((run / "partition.json").read_text("utf-8"))
        if partition.get("stage2_partition_frozen") is not True:
            raise AssertionError(partition)
        if partition.get("hash_algorithm") != "SHA-256":
            raise AssertionError(partition)
        for key in ("discovery_context_hash", "discovery_snapshot_hash",
                    "partition_hash", "input_plan_hash", "source_hash",
                    "config_hash", "calibration_hash", "solver_config_hash"):
            if not partition.get(key, "").startswith("sha256:"):
                raise AssertionError((key, partition))
        owners = []
        for segment in partition["segments"]:
            owners.extend(segment["obs_ids"])
            if not segment["parent_segment_ids"]:
                raise AssertionError(segment)
        if len(owners) != len(set(owners)):
            raise AssertionError("obs_id has multiple partition owners")
        required = {
            "support_snapshot.csv", "discovery_iterations.csv", "admm_trace.csv",
            "trajectory.tum", "imu_bias.csv", "residuals.csv",
            "factor_metadata.csv", "segments.csv", "scores_decision.csv",
        }
        missing = [name for name in required if not (run / name).exists()]
        if missing:
            raise AssertionError(f"missing Stage-1/2 artifacts: {missing}")
        with (run / "discovery_iterations.csv").open(
                newline="", encoding="utf-8") as stream:
            discovery_rows = list(csv.DictReader(stream))
        if not discovery_rows:
            raise AssertionError("automatic run has no discovery trace")
        for row in discovery_rows:
            verify_stage1_diagnostic_row(row)
        with (run / "segments.csv").open(newline="", encoding="utf-8") as stream:
            segments = list(csv.DictReader(stream))
        if len(segments) != status.get("segment_count"):
            raise AssertionError("reported segment count differs from Stage-2 export")
        capability = json.loads((run / "capability_status.json").read_text("utf-8"))
        if capability.get("gate") != "NOT_IMPLEMENTED":
            raise AssertionError(capability)
        if capability.get("fallback") != "NOT_IMPLEMENTED":
            raise AssertionError(capability)

        # Force an empty frozen partition. This still has to execute the
        # shared raw Stage-2 navigation refit and export its graph/Values;
        # candidate emptiness alone is not success.
        no_candidate_config = root / "automatic_no_candidates.yaml"
        no_candidate_text, count = re.subn(
            r"(?m)^  active_bias_min_m:.*$",
            "  active_bias_min_m: 100.0", text)
        if count != 1:
            raise AssertionError("could not override active_bias_min_m")
        no_candidate_config.write_text(no_candidate_text, encoding="utf-8")
        empty_completed = subprocess.run(
            [str(args.runner.resolve()), "--config", str(no_candidate_config),
             "--output-root", str(root), "--run-id", "no_candidates"],
            text=True, capture_output=True, check=False)
        empty_run = root / "no_candidates"
        empty_status = json.loads(
            (empty_run / "run_status.json").read_text("utf-8"))
        if empty_completed.returncode != 0:
            raise AssertionError(empty_completed.stderr)
        if empty_status.get("status") != "NO_CANDIDATES":
            raise AssertionError(empty_status)
        if empty_status.get("solver_termination") != \
                "NO_CANDIDATES_RAW_STAGE2_ALL_APPLICABLE_STOP_CONDITIONS_SATISFIED":
            raise AssertionError(empty_status)
        if empty_status.get("refit_outer_iterations", 0) < 1:
            raise AssertionError(empty_status)
        if empty_status.get("segment_count") != 0:
            raise AssertionError(empty_status)
        if empty_status.get("recoverability_status") != \
                "NOT_APPLICABLE_NO_CANDIDATES":
            raise AssertionError(empty_status)
        if empty_status.get("valid_score_exported") is not False:
            raise AssertionError(empty_status)
        empty_partition = json.loads(
            (empty_run / "partition.json").read_text("utf-8"))
        if empty_partition.get("segments") != []:
            raise AssertionError(empty_partition)
        empty_manifest = json.loads(
            (empty_run / "input_manifest.json").read_text("utf-8"))
        if empty_manifest.get("gt_or_oracle_read") is not False:
            raise AssertionError(empty_manifest)
        for artifact in ("trajectory.tum", "imu_bias.csv", "residuals.csv",
                         "factor_metadata.csv", "scores_decision.csv"):
            if not (empty_run / artifact).exists():
                raise AssertionError(f"missing raw Stage-2 artifact: {artifact}")
        with (empty_run / "factor_metadata.csv").open(
                newline="", encoding="utf-8") as stream:
            factor_rows = list(csv.DictReader(stream))
        if any(row.get("segment_id") for row in factor_rows):
            raise AssertionError("empty partition unexpectedly created C(s)")
        empty_capability = json.loads(
            (empty_run / "capability_status.json").read_text("utf-8"))
        if empty_capability.get("fallback") != "NOT_IMPLEMENTED":
            raise AssertionError(empty_capability)

        # The same empty partition with an intentionally insufficient raw
        # Stage-2 outer limit must fail without exporting an estimate.
        empty_failure_config = root / "automatic_no_candidates_refit_fail.yaml"
        empty_failure_text, count = re.subn(
            r"(?m)^  max_refit_iterations:.*$",
            "  max_refit_iterations: 1", no_candidate_text)
        if count != 1:
            raise AssertionError("could not override max_refit_iterations")
        empty_failure_config.write_text(empty_failure_text, encoding="utf-8")
        empty_failure = subprocess.run(
            [str(args.runner.resolve()), "--config", str(empty_failure_config),
             "--output-root", str(root), "--run-id", "no_candidates_refit_fail"],
            text=True, capture_output=True, check=False)
        failed_run = root / "no_candidates_refit_fail"
        failed_status = json.loads(
            (failed_run / "run_status.json").read_text("utf-8"))
        if empty_failure.returncode == 0 or failed_status.get("status") != "FAILED":
            raise AssertionError((empty_failure.returncode, failed_status))
        if failed_status.get("valid_estimate_exported") is not False:
            raise AssertionError(failed_status)
        if (failed_run / "trajectory.tum").exists():
            raise AssertionError("failed raw Stage-2 exported a trajectory")
        with (failed_run / "refit_iterations.csv").open(
                newline="", encoding="utf-8") as stream:
            failed_trace = list(csv.DictReader(stream))
        if not failed_trace or all(
                failed_trace[-1][field] == "1" for field in
                ("objective_ok", "step_ok", "kkt_ok",
                 "navigation_stationarity_ok")):
            raise AssertionError(failed_trace)
    print("T06 automatic runner no-oracle Stage-2 contract passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
