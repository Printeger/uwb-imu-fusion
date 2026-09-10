#!/usr/bin/env python3
"""Regression assertions for independent-review findings T09-R01--R08."""

import importlib.util
import json
import math
from pathlib import Path
import tempfile

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def expect_value_error(call, fragment):
    try:
        call()
    except ValueError as error:
        assert fragment in str(error), error
    else:
        raise AssertionError(f"expected ValueError containing {fragment!r}")


def batch_document(role="development"):
    return {
        "schema": "uifgo_t09_batch_v2",
        "role": role,
        "parameter_provenance": "T09_DEVELOPMENT_ENGINEERING_TEST_ONLY",
        "run_units": [{
            "run_unit_id": "recording/base/3/full",
            "recording_id": "recording",
            "base_trajectory_id": "base",
            "seed": 3,
            "prefix_identity": "full",
            "config": "input.yaml",
            "cells": [],
        }],
    }


def write_manifest(root, document):
    path = root / "batch.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def main():
    scheduler = load("t09_scheduler_review_fix",
                     ROOT / "tools/paper/run_experiments.py")
    evaluator = load("t09_evaluator_review_fix",
                     ROOT / "tools/paper/evaluate_runs.py")

    with tempfile.TemporaryDirectory(prefix="uifgo-t09-review-fix-") as tmp:
        root = Path(tmp)

        # R06: T09 is development-only until a separately authorized lock.
        validation = batch_document("validation")
        validation["run_units"][0]["cells"] = [{
            "mode": "all_range", "execution_type": "BASELINE_TRAJECTORY",
            "path": "DIRECT_COMMON_PREPARATION",
        }]
        expect_value_error(lambda: scheduler.load_manifest(
            write_manifest(root, validation)), "development-only")

        # R01/R06: canonical mode, execution type and path are a closed matrix.
        invalid = batch_document()
        invalid["run_units"][0]["cells"] = [{
            "mode": "all_range", "execution_type": "FINAL_TRAJECTORY",
            "path": "AUTO_DISCOVERY",
        }]
        expect_value_error(lambda: scheduler.load_manifest(
            write_manifest(root, invalid)), "mode/execution/path")

        # R02: operating points are distinct consumers of one named producer.
        valid = batch_document()
        valid["run_units"][0]["cells"] = [
            {"mode": "structured_debias", "execution_type": "CACHE_PRODUCER",
             "path": "FIXED_PARTITION_DEBUG", "producer_id": "fixed-p"},
            {"mode": "fit_only", "execution_type": "CACHE_DIAGNOSTIC",
             "path": "FIXED_PARTITION_DEBUG", "producer_id": "fixed-p",
             "operating_point_id": "lo", "thresholds": {"tau_gamma": 1.0}},
            {"mode": "fit_only", "execution_type": "CACHE_DIAGNOSTIC",
             "path": "FIXED_PARTITION_DEBUG", "producer_id": "fixed-p",
             "operating_point_id": "hi", "thresholds": {"tau_gamma": 2.0}},
            {"mode": "full_gate", "execution_type": "FINAL_TRAJECTORY",
             "path": "FIXED_PARTITION_DEBUG", "producer_id": "fixed-p",
             "operating_point_id": "final", "thresholds": {
                 "tau_eta": 0.0, "tau_s_m": 2.0, "tau_gamma": 3.0}},
        ]
        parsed = scheduler.load_manifest(write_manifest(root, valid))
        assert len(parsed["run_units"][0]["cells"]) == 4

        # R01: exit code zero is not final success without verified T08 output.
        final_dir = root / "final"; final_dir.mkdir()
        (final_dir / "run_status.json").write_text(json.dumps({
            "status": "OK", "exit_code": 0, "valid_estimate_exported": True,
        }))
        ok, reason = scheduler.validate_final_success(final_dir, "full_gate")
        assert not ok and reason == "FINAL_ARTIFACT_SET_INCOMPLETE"

        # R02: a published cache must contain a portable complete state.
        required = scheduler.STAGE2_REPLAY_REQUIRED_PAYLOADS
        assert {"stage2_values.csv", "partition.json", "stage2_content_identity.json",
                "scores_decision.csv", "segment_fit_scores.csv"}.issubset(required)

        # R08: loader addresses do not enter the ABI identity.
        ldd_a = "libx.so => /opt/lib/libx.so (0x00007fa111)\n/lib64/ld.so (0x7fa222)"
        ldd_b = "libx.so => /opt/lib/libx.so (0x00007fb333)\n/lib64/ld.so (0x7fb444)"
        assert scheduler.normalize_ldd_output(ldd_a) == scheduler.normalize_ldd_output(ldd_b)
        assert "0x" not in scheduler.normalize_ldd_output(ldd_a)
        provenance_a = scheduler.producer_provenance(Path("/bin/true"))
        provenance_b = scheduler.producer_provenance(Path("/bin/true"))
        assert provenance_a["producer_abi_sha256"] == provenance_b["producer_abi_sha256"]

        # R05: RPE is the SE(3) relative-pose error, not aligned displacement.
        q_identity = np.array([0.0, 0.0, 0.0, 1.0])
        q_yaw_90 = np.array([0.0, 0.0, math.sin(math.pi/4),
                             math.cos(math.pi/4)])
        est_i = (0.0, np.zeros(3), q_yaw_90)
        est_j = (1.0, np.array([1.0, 0.0, 0.0]), q_yaw_90)
        gt_i = (0.0, np.zeros(3), q_identity)
        gt_j = (1.0, np.array([1.0, 0.0, 0.0]), q_identity)
        translation, rotation = evaluator.se3_relative_pose_error(
            est_i, est_j, gt_i, gt_j)
        assert abs(translation - math.sqrt(2.0)) < 1e-12
        assert abs(rotation) < 1e-12

        rpe_run = root / "rpe"; rpe_run.mkdir()
        (rpe_run / "trajectory.tum").write_text(
            "0 0 0 0 0 0 0.7071067811865475 0.7071067811865476\n"
            "1 1 0 0 0 0 0.7071067811865475 0.7071067811865476\n"
            "2 1 1 0 0 0 0.7071067811865475 0.7071067811865476\n")
        gt = root / "rpe_gt.tum"
        gt.write_text("0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n2 1 1 0 0 0 0 1\n")
        T_GE = np.eye(4); T_GE[:3, 3] = [4.0, -2.0, 1.0]
        T_QI = np.eye(4); T_QI[:3, 3] = [0.2, 0.0, 0.0]
        trajectory = evaluator.trajectory_metrics(rpe_run, {
            "ground_truth": str(gt), "T_G_E": T_GE.tolist(),
            "T_Q_I": T_QI.tolist(), "frame_point_provenance": "TEST_FIXED",
            "time_association": {"policy": "nearest_within_tolerance",
                                 "tolerance_s": 0.0},
            "rpe_horizon_s": 1.0,
        })
        assert abs(trajectory["RPE_rmse_m"]["value"] - math.sqrt(2.0)) < 1e-12
        assert trajectory["RPE_rotation_rmse_rad"]["value"] == 0.0

        # R07: comparability-invalid cells are quarantined from aggregates.
        assert evaluator.cell_is_comparable({"status": "COMPARABILITY_INVALID"}) is False
        assert evaluator.cell_is_comparable({"status": "COMPLETE"}) is True

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
