#!/usr/bin/env python3
import json
import hashlib
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]


def main():
    with tempfile.TemporaryDirectory(prefix="uifgo-t09-eval-") as temporary:
        root = Path(temporary)
        good = root / "good"; good.mkdir()
        bad = root / "bad"; bad.mkdir()
        bias = root / "bias"; bias.mkdir()
        trajectory = "0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n2 2 0 0 0 0 0 1\n"
        (good / "trajectory.tum").write_text(trajectory)
        (good / "observations.csv").write_text("obs_id,valid,planned\n1,1,1\n2,1,0\n")
        (good / "baseline_factor_audit.csv").write_text("obs_id,final_use,reason\n1,1,RETAINED\n")
        (good / "run_status.json").write_text(json.dumps({
            "status": "OK", "valid_estimate_exported": True}))
        (bad / "run_status.json").write_text(json.dumps({
            "status": "ESTIMATION_FAILED", "valid_estimate_exported": False}))
        (bias / "trajectory.tum").write_text(trajectory)
        (bias / "observations.csv").write_text(
            "obs_id,valid,planned\n1,1,1\n2,1,1\n")
        (bias / "factor_metadata.csv").write_text("obs_id\n1\n2\n")
        (bias / "stage1_observation_bias.csv").write_text(
            "obs_id,candidate,bias_m\n1,1,0.1\n2,1,0.5\n")
        (bias / "run_status.json").write_text(json.dumps({
            "status": "OK", "valid_estimate_exported": True,
            "covariance_status":
                "NOT_APPLICABLE_REGULARIZED_STAGE1_NON_GAUSSIAN"}))
        gt = root / "gt.tum"; gt.write_text(trajectory)
        bias_truth = root / "bias_truth.csv"
        bias_truth.write_text("obs_id,total_bias_m\n1,0.1\n2,0.2\n")
        batch = {
            "schema": "uifgo_t09_batch_manifest_v1", "status": "COMPLETE_WITH_RUN_FAILURES",
            "cells": [
                {"cell_id": "a", "run_unit_id": "u", "canonical_mode": "all_range",
                 "execution_type": "BASELINE_TRAJECTORY", "status": "COMPLETE",
                 "run_directory": str(good), "attempts": [{"measured_wall_seconds": 1.0,
                                                              "measured_peak_rss_kib": 2}]},
                {"cell_id": "b", "run_unit_id": "v", "canonical_mode": "all_range",
                 "execution_type": "BASELINE_TRAJECTORY", "status": "COMPLETE_WITH_RUN_FAILURE",
                 "run_directory": str(bad), "attempts": [{"measured_wall_seconds": 2.0,
                                                            "measured_peak_rss_kib": 3}]},
                {"cell_id": "c", "run_unit_id": "u", "canonical_mode": "fit_only",
                 "execution_type": "CACHE_DIAGNOSTIC", "status": "PARENT_CACHE_UNAVAILABLE",
                 "attempts": []},
                {"cell_id": "d", "run_unit_id": "w",
                 "canonical_mode": "structured_bias_only",
                 "execution_type": "STAGE1_TRAJECTORY", "status": "COMPLETE",
                 "run_directory": str(bias), "attempts": []},
            ]}
        batch_path = root / "batch.json"; batch_path.write_text(json.dumps(batch))
        eye = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
        evaluation = root / "evaluation.yaml"
        evaluation.write_text(yaml.safe_dump({
            "schema": "uifgo_t09_evaluation_v1",
            "run_units": {"u": {"ground_truth": "gt.tum", "T_G_E": eye,
                "T_Q_I": eye, "frame_point_provenance": "TEST_FIXED",
                "ground_truth_sha256": "sha256:" + hashlib.sha256(
                    trajectory.encode()).hexdigest(),
                "units": "m_s_rad", "time_base": "TEST_SECONDS",
                "bias_truth_scope": "INJECTED_COMPONENT_ONLY",
                "time_association": {"policy": "nearest_within_tolerance",
                                     "tolerance_s": 0.0}, "rpe_horizon_s": 1.0},
                          "w": {"bias_truth_scope": "TOTAL_SYNTHETIC_LATENT",
                                "bias_truth": "bias_truth.csv",
                                "bias_truth_provenance": "TEST_GENERATED_TOTAL",
                                "epsilon_bad_m": 0.1}},
        }))
        completed = subprocess.run([
            sys.executable, "-B", str(ROOT / "tools/paper/evaluate_runs.py"),
            "--batch-manifest", str(batch_path), "--evaluation-manifest",
            str(evaluation)], text=True, capture_output=True)
        assert completed.returncode == 0, completed.stderr
        result = json.loads((root / "evaluation.json").read_text())
        failures = result["aggregate"]["trajectory_failure_fraction"]
        failure = next(value for key, value in failures.items()
                       if key.startswith("all_range/"))
        assert failure["numerator"] == 1 and failure["denominator"] == 2
        rows = {row["cell_id"]: row for row in result["run_metrics"]}
        assert rows["a"]["trajectory"]["raw_frame_ATE_rmse_m"]["value"] == 0.0
        assert rows["a"]["coverage"]["overall_retained_fraction"]["value"] == 0.5
        assert rows["b"]["trajectory"]["raw_frame_ATE_rmse_m"]["status"] == "UNAVAILABLE_ESTIMATION_FAILURE"
        assert rows["a"]["bias_correction"]["bad_correction_rate"]["status"] == "UNAVAILABLE_INCOMPLETE_BIAS_TRUTH"
        diagnostic = result["aggregate"]["diagnostic_cell_unavailable_fraction"]
        assert diagnostic["numerator"] == 1 and diagnostic["denominator"] == 1
        bias_result = rows["d"]["bias_correction"]
        assert abs(bias_result["candidate_bias_field_RMSE"]["value"] -
                   (0.09 / 2) ** 0.5) < 1e-12
        assert bias_result["bad_correction_rate"]["numerator"] == 1
        assert bias_result["bad_correction_rate"]["denominator"] == 2
        assert bias_result["good_correction_rejection_rate"]["value"] == 0.0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
