#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_scheduler_accepts_and_canonicalizes_real_anchor_ids():
    scheduler = load("step2_scheduler", ROOT / "tools/paper/run_experiments.py")
    with tempfile.TemporaryDirectory(prefix="ie0911-step2-manifest-") as tmp:
        path = Path(tmp) / "manifest.yaml"
        path.write_text(yaml.safe_dump({
            "schema": "uifgo_t09_batch_v2",
            "role": "development",
            "parameter_provenance": "IE0911_TEST_ONLY",
            "run_units": [{
                "run_unit_id": "low_redundancy", "recording_id": "r",
                "base_trajectory_id": "r", "seed": 0,
                "prefix_identity": "full", "config": "unused.yaml",
                "anchor_ids": [4, 1, 3, 0],
                "cells": [{"mode": "robust_cauchy",
                           "execution_type": "BASELINE_TRAJECTORY",
                           "path": "DIRECT_COMMON_PREPARATION",
                           "robust_scale": 2.3849}],
            }],
        }), encoding="utf-8")
        manifest = scheduler.load_manifest(path)
        assert manifest["run_units"][0]["anchor_ids"] == [0, 1, 3, 4]

        captured = {}
        class Completed:
            returncode = 0
            stdout = ""
            stderr = ""
        def fake_run(command, **_kwargs):
            captured["command"] = command
            return Completed()
        scheduler.subprocess.run = fake_run
        output = Path(tmp) / "runs"; output.mkdir()
        scheduler.runner_call(Path("/runner"), Path("/config"), output,
                              "run", None, "BASELINE_TRAJECTORY", None,
                              None, [0, 1, 3, 4], {})
        index = captured["command"].index("--anchor-ids")
        assert captured["command"][index + 1] == "0,1,3,4"


def test_paired_metrics_use_common_gt_points_and_positive_improvement():
    evaluator = load("step2_evaluator", ROOT / "tools/paper/evaluate_runs.py")
    with tempfile.TemporaryDirectory(prefix="ie0911-step2-pair-") as tmp:
        root = Path(tmp)
        method = root / "method"; method.mkdir()
        suppress = root / "suppress"; suppress.mkdir()
        gt = root / "gt.tum"
        gt.write_text(
            "0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n"
            "2 2 1 0 0 0 0 1\n3 3 1 0 0 0 0 1\n", encoding="utf-8")
        (method / "trajectory.tum").write_text(
            "0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n"
            "2 2 1 0 0 0 0 1\n3 3 1 0 0 0 0 1\n", encoding="utf-8")
        (suppress / "trajectory.tum").write_text(
            "0 0 0 0 0 0 0 1\n1 1 0.2 0 0 0 0 1\n"
            "2 2 1.4 0 0 0 0 1\n3 3 0.7 0 0 0 0 1\n", encoding="utf-8")
        scenario = {
            "ground_truth": str(gt),
            "time_association": {"policy": "nearest_within_tolerance",
                                 "tolerance_s": 0.0},
        }
        result = evaluator.paired_aligned_metrics(method, suppress, scenario)
        assert result["status"] == "AVAILABLE"
        assert result["matched_gt_count"] == 4
        assert result["rmse_improvement_vs_suppress_m"] > 0.0
        assert result["p95_improvement_vs_suppress_m"] > 0.0
        assert result["alignment"] == "SE3_SCALE_FIXED_ONE"


if __name__ == "__main__":
    test_scheduler_accepts_and_canonicalizes_real_anchor_ids()
    test_paired_metrics_use_common_gt_points_and_positive_improvement()
