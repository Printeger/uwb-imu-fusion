#!/usr/bin/env python3
"""Regressions for T09 rereview R01-A/B, R02-A, R03-A/B/C, R04-A."""

import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha(path):
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    scheduler = load("t09_scheduler_rereview",
                     ROOT / "tools/paper/run_experiments.py")
    evaluator = load("t09_evaluator_rereview",
                     ROOT / "tools/paper/evaluate_runs.py")

    # R01-A: neighboring binary64 policy points remain distinct request input.
    lo = {"tau_gamma": 1.0000001}
    hi = {"tau_gamma": 1.0000002}
    lo_hash = scheduler.lossless_thresholds_sha256("fit_only", lo)
    hi_hash = scheduler.lossless_thresholds_sha256("fit_only", hi)
    assert lo_hash != hi_hash
    request = {
        "stage2_cache_id": "t09stage2cache-sha256:shared",
        "canonical_mode": "fit_only",
        "policy_version": "T09_FIT_ONLY_FINAL_AUDIT_V1",
        "thresholds_sha256": lo_hash,
        "threshold_provenance": "TEST_ONLY",
        "final_refit_score_config_sha256": "sha256:final",
        "solver_sha256": "sha256:solver",
        "common_preparation_id": "t09common-sha256:shared",
    }
    lo_request = scheduler.final_request_identity(request)
    request["thresholds_sha256"] = hi_hash
    hi_request = scheduler.final_request_identity(request)
    assert lo_request != hi_request
    request["final_refit_score_config_sha256"] = "sha256:changed-final"
    assert hi_request != scheduler.final_request_identity(request)
    request["final_refit_score_config_sha256"] = "sha256:final"
    request["solver_sha256"] = "sha256:changed-solver"
    assert hi_request != scheduler.final_request_identity(request)

    # R02-A: gate-only edits reuse Stage 2, discovery edits do not.
    base = {"nlos": {"mode": "automatic_discovery", "lambda_l1": 0.05,
                     "lambda_tv": 0.1, "tau_gamma": 1.0,
                     "gate_parameter_provenance": "TEST_ONLY",
                     "final_inference_enabled": False}}
    gate = json.loads(json.dumps(base)); gate["nlos"]["tau_gamma"] = 2.0
    discovery = json.loads(json.dumps(base)); discovery["nlos"]["lambda_l1"] = 0.1
    v1 = json.loads(json.dumps(base))
    v1["nlos"]["discovery_conditional_navigation_policy"] = \
        "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_V1"
    v2 = json.loads(json.dumps(base))
    v2["nlos"]["discovery_conditional_navigation_policy"] = \
        "GTSAM_CHECK_AND_NAVIGATION_STATIONARITY_CONTINUE_LAMBDA_SEARCH_V2"
    assert scheduler.stage2_producer_config_identity(base, Path(".")) == \
        scheduler.stage2_producer_config_identity(gate, Path("."))
    assert scheduler.stage2_producer_config_identity(base, Path(".")) != \
        scheduler.stage2_producer_config_identity(discovery, Path("."))
    producer_ids = {
        scheduler.stage2_producer_config_identity(config, Path("."))
        for config in (base, v1, v2)
    }
    assert len(producer_ids) == 3

    with tempfile.TemporaryDirectory(prefix="uifgo-t09-rereview-") as tmp:
        root = Path(tmp)
        evaluation = root / "evaluation.yaml"
        evaluation.write_text(yaml.safe_dump({
            "schema": "uifgo_t09_evaluation_v1", "run_units": {}}))

        # R03-A/C: upstream and ordinary estimator failures remain rows and
        # run-unit denominators, separately stratified by point/path.
        failed = root / "failed"; failed.mkdir()
        (failed / "run_status.json").write_text(json.dumps({
            "status": "FAILED", "exit_code": 1,
            "valid_estimate_exported": False, "reason": "LM_FAILED"}))
        replay_rejected = root / "replay_rejected"; replay_rejected.mkdir()
        (replay_rejected / "run_status.json").write_text(json.dumps({
            "status": "FAILED", "exit_code": 1,
            "valid_estimate_exported": False,
            "reason": "CACHE_STAGE1_CONFIG_INCOMPATIBLE"}))
        failed_final = root / "failed_final"; failed_final.mkdir()
        failed_inference = "t08inference-sha256:failed-fallback"
        failed_request = "t09finalrequest-sha256:failed-fallback"
        failed_files = {
            "final_masks.csv":
                "inference_id,obs_id,candidate,decision_use,final_use,reason_code\n"
                f"{failed_inference},1,1,1,0,USE\n",
            "final_factor_audit.csv":
                "inference_id,obs_id,classification,final_factor_count,expected_count,ok\n"
                f"{failed_inference},1,SUPPRESSED_CANDIDATE,0,0,0\n",
            "decisions.csv":
                "inference_id,group_id,eligible,decision\n"
                f"{failed_inference},g0,1,USE\n",
            "final_content_identity.json": json.dumps({
                "inference_id": failed_inference,
                "graph_linearization_sha256": "sha256:g",
                "values_sha256": "sha256:v", "context_sha256": "sha256:c"}),
            "final_inference_summary.json": json.dumps({
                "inference_id": failed_inference, "valid_estimate": False,
                "factor_audit_status": "FAILED", "fallback_attempt_count": 1,
                "recovery_attempt_execution_status": "SUCCEEDED",
                "recovery_attempt_solver_status": "CONVERGED",
                "recovery_attempt_acceptance_audit_status": "FAILED"}),
        }
        for name, content in failed_files.items():
            (failed_final / name).write_text(content)
        (failed_final / "run_status.json").write_text(json.dumps({
            "status": "ESTIMATION_FAILED", "exit_code": 1,
            "valid_estimate_exported": False,
            "inference_id": failed_inference, "final_request_id": failed_request}))
        (failed_final / "run_manifest.json").write_text(json.dumps({
            "canonical_mode": "full_gate", "operating_point_id": "failed",
            "inference_id": failed_inference, "final_request_id": failed_request,
            "export_verification_status": "VERIFIED",
            "artifact_sha256": [f"{name}={sha(failed_final / name)}"
                                for name in failed_files]}))
        cells = [
            {"cell_id": "missing-a", "run_unit_id": "u",
             "canonical_mode": "fit_only", "execution_type": "FINAL_TRAJECTORY",
             "path": "AUTO_DISCOVERY", "operating_point_id": "a",
             "producer_id": "p", "status": "PARENT_CACHE_UNAVAILABLE",
             "reason": "NO_ADMITTED_STAGE2_CACHE", "attempts": []},
            {"cell_id": "missing-b", "run_unit_id": "u",
             "canonical_mode": "fit_only", "execution_type": "FINAL_TRAJECTORY",
             "path": "FIXED_PARTITION_DEBUG", "operating_point_id": "b",
             "producer_id": "q", "status": "PARENT_CACHE_UNAVAILABLE",
             "reason": "NO_ADMITTED_STAGE2_CACHE", "attempts": []},
            {"cell_id": "failed-baseline", "run_unit_id": "v",
             "canonical_mode": "all_range", "execution_type": "BASELINE_TRAJECTORY",
             "path": "DIRECT_COMMON_PREPARATION", "operating_point_id": "default",
             "status": "COMPLETE_WITH_RUN_FAILURE",
             "run_directory": str(failed), "attempts": []},
            {"cell_id": "failed-final", "run_unit_id": "w",
             "canonical_mode": "full_gate", "execution_type": "FINAL_TRAJECTORY",
             "path": "AUTO_DISCOVERY", "operating_point_id": "failed",
             "status": "COMPLETE_WITH_RUN_FAILURE",
             "run_directory": str(failed_final), "attempts": []},
            {"cell_id": "replay-rejected", "run_unit_id": "x",
             "canonical_mode": "fit_only", "execution_type": "FINAL_TRAJECTORY",
             "path": "AUTO_DISCOVERY", "operating_point_id": "rejected",
             "status": "COMPLETE_WITH_RUN_FAILURE",
             "run_directory": str(replay_rejected), "attempts": []},
        ]
        batch = root / "failure_batch.json"
        batch.write_text(json.dumps({
            "schema": "uifgo_t09_batch_manifest_v2",
            "status": "COMPLETE_WITH_RUN_FAILURES", "cells": cells}))
        completed = subprocess.run([
            sys.executable, "-B", str(ROOT / "tools/paper/evaluate_runs.py"),
            "--batch-manifest", str(batch), "--evaluation-manifest",
            str(evaluation)], text=True, capture_output=True)
        assert completed.returncode == 0, completed.stderr
        result = json.loads((root / "evaluation.json").read_text())
        rows = {row["cell_id"]: row for row in result["run_metrics"]}
        assert set(rows) == {cell["cell_id"] for cell in cells}
        assert rows["missing-a"]["trajectory"]["RPE_rmse_m"]["status"] == \
            "UNAVAILABLE_ESTIMATION_FAILURE"
        strata = result["aggregate"]["final_estimation_failure_fraction"]
        assert len(strata) == 4
        assert all(value["numerator"] == value["denominator"] == 1
                   and value["unit"] == "run_unit" for value in strata.values())
        baseline = result["aggregate"]["trajectory_failure_fraction"]
        assert any(value["numerator"] == value["denominator"] == 1
                   for value in baseline.values())
        assert rows["replay-rejected"]["coverage"][
            "candidate_use_coverage"]["status"] == "UNAVAILABLE_UPSTREAM_FAILURE"
        recovery = next(iter(result["aggregate"][
            "recovery_failure_fraction"].values()))
        fallback_attempt = next(iter(result["aggregate"][
            "fallback_attempt_fraction"].values()))
        fallback_success = next(iter(result["aggregate"][
            "fallback_success_fraction"].values()))
        assert recovery["numerator"] == recovery["denominator"] == 1
        assert fallback_attempt["numerator"] == fallback_attempt["denominator"] == 1
        assert fallback_success["numerator"] == 0 and fallback_success["denominator"] == 1

        # R03-B: a same-inference-ID edit is rejected by the sealed hashes.
        final = root / "final"; final.mkdir()
        inference = "t08inference-sha256:sealed"
        files = {
            "trajectory.tum": "0 0 0 0 0 0 0 1\n",
            "final_masks.csv":
                "inference_id,obs_id,candidate,decision_use,final_use,segment_id,group_id\n"
                f"{inference},1,1,1,1,s0,g0\n",
            "final_factor_audit.csv":
                "inference_id,obs_id,classification,final_factor_count,expected_count,ok\n"
                f"{inference},1,ACCEPTED_CANDIDATE_RAW_WITH_LIVE_C,1,1,1\n",
            "decisions.csv":
                "inference_id,group_id,eligible,decision\n"
                f"{inference},g0,1,USE\n",
            "segment_bias.csv":
                "inference_id,segment_id,amplitude_m\n"
                f"{inference},s0,0.4\n",
            "final_content_identity.json": json.dumps({
                "inference_id": inference,
                "graph_linearization_sha256": "sha256:g",
                "values_sha256": "sha256:v",
                "context_sha256": "sha256:c"}),
            "final_inference_summary.json": json.dumps({
                "inference_id": inference, "valid_estimate": True,
                "factor_audit_status": "OK", "fallback_attempt_count": 0}),
        }
        for name, content in files.items():
            (final / name).write_text(content)
        (final / "run_status.json").write_text(json.dumps({
            "status": "OK", "exit_code": 0, "valid_estimate_exported": True,
            "inference_id": inference, "final_request_id": "t09finalrequest-sha256:x"}))
        artifact_sha = [f"{name}={sha(final / name)}" for name in files]
        (final / "run_manifest.json").write_text(json.dumps({
            "canonical_mode": "fit_only", "inference_id": inference,
            "final_request_id": "t09finalrequest-sha256:x",
            "export_verification_status": "VERIFIED",
            "artifact_sha256": artifact_sha}))
        with (final / "final_masks.csv").open(newline="") as stream:
            mask_rows = list(csv.DictReader(stream)); fields = mask_rows[0].keys()
        mask_rows[0]["final_use"] = "0"
        with (final / "final_masks.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader(); writer.writerows(mask_rows)
        tampered_batch = root / "tampered_batch.json"
        tampered_batch.write_text(json.dumps({
            "schema": "uifgo_t09_batch_manifest_v2", "status": "COMPLETE",
            "cells": [{"cell_id": "tampered", "run_unit_id": "t",
                "canonical_mode": "fit_only", "execution_type": "FINAL_TRAJECTORY",
                "path": "AUTO_DISCOVERY", "operating_point_id": "x",
                "producer_id": "p", "status": "COMPLETE",
                "run_directory": str(final), "attempts": []}]}))
        rejected = subprocess.run([
            sys.executable, "-B", str(ROOT / "tools/paper/evaluate_runs.py"),
            "--batch-manifest", str(tampered_batch), "--evaluation-manifest",
            str(evaluation), "--output", str(root / "tampered.json")],
            text=True, capture_output=True)
        assert rejected.returncode == 2, rejected.stderr
        assert "artifact SHA-256 mismatch" in rejected.stderr, rejected.stderr

        # R04-A: decision candidate risk is immutable; accepted final risk
        # requires a complete final C-state mapping and never falls back.
        cache = root / "risk_cache"; cache.mkdir()
        final_bias = root / "risk_final"; final_bias.mkdir()
        (cache / "partition.json").write_text(json.dumps({"segments": [
            {"segment_id": "a", "segment_ordinal": 0, "obs_ids": [1]},
            {"segment_id": "b", "segment_ordinal": 1, "obs_ids": [2]}]}))
        (cache / "segments.csv").write_text(
            "segment_id,segment_ordinal,amplitude_m\na,0,0.5\nb,1,0.5\n")
        (final_bias / "final_masks.csv").write_text(
            "obs_id,candidate,decision_use,final_use,segment_id\n"
            "1,1,1,1,a\n2,1,0,0,b\n")
        truth = root / "truth.csv"
        truth.write_text("obs_id,total_bias_m\n1,0.4\n2,0.2\n")
        scenario = {"bias_truth_scope": "TOTAL_SYNTHETIC_LATENT",
                    "bias_truth": str(truth),
                    "bias_truth_provenance": "SCHEMA_TEST_ONLY",
                    "epsilon_bad_m": 0.1}
        candidate_values = []
        for value in (0.4, 0.9):
            (final_bias / "segment_bias.csv").write_text(
                f"segment_id,amplitude_m\na,{value}\n")
            metrics = evaluator.bias_correction_metrics(
                final_bias, scenario, "FINAL_TRAJECTORY", cache)
            candidate_values.append(metrics["candidate_bias_field_RMSE"]["value"])
        expected = math.sqrt((0.1 ** 2 + 0.3 ** 2) / 2)
        assert all(abs(value - expected) < 1e-12 for value in candidate_values)
        (final_bias / "segment_bias.csv").write_text("segment_id,amplitude_m\n")
        missing = evaluator.bias_correction_metrics(
            final_bias, scenario, "FINAL_TRAJECTORY", cache)
        assert missing["accepted_bias_field_RMSE"]["status"] == \
            "UNAVAILABLE_FINAL_AMPLITUDE_MAPPING"

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
