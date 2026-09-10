#!/usr/bin/env python3
"""Exporter-schema fixture for T09-R03/R04 accounting and cache joins."""

import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
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


def reseal(run, artifact):
    manifest_path = run / "run_manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["artifact_sha256"] = [
        f"{artifact}={sha(run / artifact)}"
        if entry.startswith(f"{artifact}=") else entry
        for entry in manifest["artifact_sha256"]]
    manifest_path.write_text(json.dumps(manifest))


def rewrite_csv(path, mutate):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        fields, rows = reader.fieldnames, list(reader)
    mutate(rows)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    scheduler = load("t09_scheduler_artifact_graph",
                     ROOT / "tools/paper/run_experiments.py")
    with tempfile.TemporaryDirectory(prefix="uifgo-t09-artifact-graph-") as tmp:
        root = Path(tmp)
        cache = root / "cache"; cache.mkdir()
        files = {
            "observations.csv": (
                "obs_id,valid,planned\n1,1,1\n2,1,1\n3,1,1\n4,1,0\n"),
            "input_manifest.json": json.dumps({
                "source_hash_sha256": "sha256:source"}) + "\n",
            "partition.json": json.dumps({"segments": [{
                "segment_id": "s0", "segment_ordinal": 0,
                "obs_ids": [1, 2]}]}) + "\n",
            "support_snapshot.csv": "obs_id,bias_m\n1,0.5\n2,0.5\n",
            "segments.csv": "segment_id,segment_ordinal,amplitude_m,short_support_debug,boundary\ns0,0,0.5,0,0\n",
            "factor_metadata.csv": "factor_index,obs_id,factor_type\n0,1,UWB\n1,2,UWB\n2,3,UWB\n",
            "refit_iterations.csv": "iteration,status\n0,CONVERGED\n",
            "groups.csv": "group_id,start_time,end_time,segment_ordinals,eligible,status\ng0,0,1,0,1,OK\n",
            "scores_decision.csv": "group_id,status,eligible,valid_score_exported,score_availability,eta,s_m,s_is_infinite,linearization_id\ng0,OK,1,1,AVAILABLE_FINITE,1,0,0,lin0\n",
            "segment_fit_scores.csv": "segment_id,gamma\ns0,0\n",
            "trajectory.tum": "0 0 0 0 0 0 0 1\n",
            "imu_bias.csv": "key,bax,bay,baz,bgx,bgy,bgz\nb0,0,0,0,0,0,0\n",
            "stage2_values.csv": "key,type,value\nx0,Pose3,test\n",
            "stage2_content_identity.json": json.dumps({
                "graph_linearization_sha256": "sha256:graph",
                "values_sha256": "sha256:values"}) + "\n",
            "stage2_producer_context.json": json.dumps({
                "schema": "uifgo_t09_stage2_producer_context_v1",
                "stage1_config_sha256": "sha256:stage1",
                "stage2_refit_config_sha256": "sha256:stage2-refit",
                "stage3_score_config_sha256": "sha256:stage3-score"}) + "\n",
        }
        for name, content in files.items():
            (cache / name).write_text(content, encoding="utf-8")
        assert (scheduler.STAGE2_REPLAY_REQUIRED_PAYLOADS |
                {"trajectory.tum", "imu_bias.csv"}).issubset(files)
        cache_manifest = {
            "schema": "uifgo_t09_stage2_cache_v2",
            "cache_namespace": "FIXED_PARTITION_DEBUG",
            "debug_label": "RQ3_FIXED_PARTITION_DIAGNOSTIC_DEBUG_ONLY",
            "common_preparation_id": "t09common-sha256:test",
            "source_identity": "sha256:source",
            "producer_common_config_sha256": "sha256:common-config",
            "producer_stage2_config_sha256": "sha256:stage2-config",
            "stage1_config_sha256": "sha256:stage1",
            "stage2_refit_config_sha256": "sha256:stage2-refit",
            "stage3_score_config_sha256": "sha256:stage3-score",
            "support_partition_sha256": sha(cache / "segments.csv"),
            "observation_mapping_sha256": sha(cache / "observations.csv"),
            "stage2_graph_linearization_sha256": "sha256:graph",
            "stage2_values_sha256": "sha256:values",
            "factor_metadata_sha256": sha(cache / "factor_metadata.csv"),
            "stage2_trace_sha256": sha(cache / "refit_iterations.csv"),
            "score_table_sha256": sha(cache / "scores_decision.csv"),
            "producer_commit": "test-commit",
            "producer_binary_sha256": "sha256:binary",
            "producer_abi_sha256": "sha256:abi",
            "producer_toolchain_sha256": "sha256:toolchain",
            "stage2_status": "CONVERGED",
            "score_status": "COMPLETE",
            "payloads": [{"name": name, "sha256": sha(cache / name)}
                         for name in sorted(files)],
        }
        cache_id = scheduler.stage2_cache_identity(cache_manifest)
        cache_manifest["cache_id"] = cache_id
        cache_manifest_path = cache / "stage2_cache_manifest.json"
        cache_manifest_path.write_text(json.dumps(cache_manifest), encoding="utf-8")

        diagnostic = root / "diagnostic"; diagnostic.mkdir()
        (diagnostic / "diagnostic.json").write_text(json.dumps({
            "cache_id": cache_id, "cache_namespace": "FIXED_PARTITION_DEBUG",
            "decisions": [{"group_id": "g0", "decision": "USE"}],
            "candidate_use_coverage": {"value": 1.0, "status": "AVAILABLE",
                "numerator": 2, "denominator": 2, "unit": "observation", "reason": ""},
            "eligible_use_coverage": {"value": 1.0, "status": "AVAILABLE",
                "numerator": 2, "denominator": 2, "unit": "observation", "reason": ""},
            "overall_retained_fraction": {"value": None,
                "status": "NOT_APPLICABLE_NO_FINAL_GRAPH", "numerator": None,
                "denominator": None, "unit": "observation", "reason": "diagnostic"},
        }))

        final = root / "final"; final.mkdir()
        inference = "t08inference-sha256:test"
        (final / "trajectory.tum").write_text(
            "0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n2 1 1 0 0 0 0 1\n")
        (final / "run_status.json").write_text(json.dumps({
            "status": "OK", "exit_code": 0, "valid_estimate_exported": True,
            "final_request_id": "t09finalrequest-sha256:test",
            "inference_id": inference,
        }))
        (final / "final_inference_summary.json").write_text(json.dumps({
            "inference_id": inference, "valid_estimate": True,
            "factor_audit_status": "OK",
            "fallback_attempt_count": 0,
            "recovery_attempt_execution_status": "SUCCEEDED",
            "recovery_attempt_solver_status": "CONVERGED",
            "recovery_attempt_acceptance_audit_status": "PASSED",
        }))
        (final / "decisions.csv").write_text(
            "inference_id,group_id,eligible,decision\n%s,g0,1,USE\n" % inference)
        (final / "final_masks.csv").write_text(
            "inference_id,obs_id,candidate,noncandidate_reference,decision_use,final_use,fallback_use,segment_id,group_id,reason_code\n"
            f"{inference},1,1,0,1,1,0,s0,g0,USE\n"
            f"{inference},2,1,0,1,0,0,s0,g0,USE\n"
            f"{inference},3,0,1,0,1,0,,,NONCANDIDATE_REFERENCE\n"
            f"{inference},4,0,0,0,0,0,,,NOT_IN_FROZEN_VALID_PLAN\n")
        (final / "final_factor_audit.csv").write_text(
            "inference_id,obs_id,classification,final_factor_count,expected_count,ok\n"
            f"{inference},1,ACCEPTED_CANDIDATE_RAW_WITH_LIVE_C,1,1,1\n"
            f"{inference},2,SUPPRESSED_CANDIDATE,0,0,1\n"
            f"{inference},3,NONCANDIDATE_REFERENCE,1,1,1\n")
        (final / "segment_bias.csv").write_text(
            "inference_id,segment_id,segment_ordinal,c_key,amplitude_m\n"
            f"{inference},s0,0,c0,0.4\n")
        (final / "final_content_identity.json").write_text(json.dumps({
            "inference_id": inference,
            "graph_linearization_sha256": "sha256:graph",
            "values_sha256": "sha256:values",
            "context_sha256": "sha256:context"}))
        sealed_names = [
            "trajectory.tum", "decisions.csv", "final_masks.csv",
            "final_factor_audit.csv", "segment_bias.csv",
            "final_inference_summary.json", "final_content_identity.json"]
        (final / "run_manifest.json").write_text(json.dumps({
            "canonical_mode": "full_gate", "stage2_cache_id": cache_id,
            "final_request_id": "t09finalrequest-sha256:test",
            "inference_id": inference, "export_verification_status": "VERIFIED",
            "artifact_sha256": [f"{name}={sha(final / name)}"
                                for name in sealed_names]}))

        truth = root / "truth.csv"
        truth.write_text("obs_id,total_bias_m\n1,0.4\n2,0.2\n")
        evaluation = root / "evaluation.yaml"
        evaluation.write_text(yaml.safe_dump({
            "schema": "uifgo_t09_evaluation_v1", "run_units": {"u": {
                "bias_truth_scope": "TOTAL_SYNTHETIC_LATENT",
                "bias_truth": "truth.csv", "bias_truth_provenance": "TEST_TOTAL",
                "epsilon_bad_m": 0.1}}}))
        cells = []
        for cell_id, mode, execution, directory in (
                ("d", "fit_only", "CACHE_DIAGNOSTIC", diagnostic),
                ("f", "full_gate", "FINAL_TRAJECTORY", final)):
            cells.append({"cell_id": cell_id, "run_unit_id": "u",
                "canonical_mode": mode, "execution_type": execution,
                "status": "COMPLETE", "run_directory": str(directory),
                "cache_id": cache_id,
                "parent_cache_manifest": str(cache_manifest_path), "attempts": []})
        batch = root / "batch.json"
        batch.write_text(json.dumps({"schema": "uifgo_t09_batch_manifest_v2",
            "status": "COMPLETE", "cells": cells}))
        completed = subprocess.run([sys.executable, "-B",
            str(ROOT / "tools/paper/evaluate_runs.py"), "--batch-manifest",
            str(batch), "--evaluation-manifest", str(evaluation)],
            text=True, capture_output=True)
        assert completed.returncode == 0, completed.stderr
        result = json.loads((root / "evaluation.json").read_text())
        rows = {row["cell_id"]: row for row in result["run_metrics"]}
        assert rows["f"]["coverage"]["candidate_use_coverage"]["value"] == 1.0
        assert rows["f"]["coverage"]["overall_retained_fraction"]["value"] == 1/2
        assert rows["f"]["bias_correction"]["accepted_bias_field_RMSE"]["value"] == 0.0
        assert rows["f"]["bias_correction"]["good_correction_rejection_rate"]["value"] == 0.0
        assert rows["d"]["support"]["segment_count"] == 1
        assert rows["d"]["bias_correction"]["bad_correction_rate"]["numerator"] == 1
        for name in ("recovery_failure_fraction",
                     "recovery_solve_failure_fraction",
                     "recovery_final_audit_failure_fraction"):
            values = result["aggregate"][name]
            item = next(value for key, value in values.items()
                        if key.startswith("full_gate/"))
            assert item["value"] == 0.0

        def rejected_variant(name, mutate, expected_error):
            variant = root / name
            shutil.copytree(final, variant)
            artifact = mutate(variant)
            if artifact:
                reseal(variant, artifact)
            negative_batch = root / f"{name}_batch.json"
            negative_batch.write_text(json.dumps({
                "schema": "uifgo_t09_batch_manifest_v2", "status": "COMPLETE",
                "cells": [{"cell_id": name, "run_unit_id": "u",
                    "canonical_mode": "full_gate",
                    "execution_type": "FINAL_TRAJECTORY", "status": "COMPLETE",
                    "run_directory": str(variant), "cache_id": cache_id,
                    "parent_cache_manifest": str(cache_manifest_path),
                    "attempts": []}]}))
            output = root / f"{name}_evaluation.json"
            completed = subprocess.run([
                sys.executable, "-B", str(ROOT / "tools/paper/evaluate_runs.py"),
                "--batch-manifest", str(negative_batch),
                "--evaluation-manifest", str(evaluation), "--output", str(output)],
                text=True, capture_output=True)
            assert completed.returncode == 2, (name, completed.stderr)
            assert expected_error in completed.stderr, (name, completed.stderr)
            assert not output.exists(), name

        def orphan_audit(run):
            path = run / "final_factor_audit.csv"
            def mutate(rows):
                rows.append({"inference_id": inference, "obs_id": "orphan",
                    "classification": "NONCANDIDATE_REFERENCE",
                    "final_factor_count": "1", "expected_count": "1", "ok": "1"})
            rewrite_csv(path, mutate)
            return path.name

        def missing_audit(run):
            path = run / "final_factor_audit.csv"
            rewrite_csv(path, lambda rows: rows.pop())
            return path.name

        def duplicate_audit(run):
            path = run / "final_factor_audit.csv"
            rewrite_csv(path, lambda rows: rows.append(dict(rows[0])))
            return path.name

        def wrong_classification(run):
            path = run / "final_factor_audit.csv"
            rewrite_csv(path, lambda rows: rows[2].update(
                classification="SUPPRESSED_CANDIDATE"))
            return path.name

        def contradictory_expected_count(run):
            path = run / "final_factor_audit.csv"
            rewrite_csv(path, lambda rows: rows[2].update(expected_count="0"))
            return path.name

        def contradictory_flags(run):
            path = run / "final_masks.csv"
            rewrite_csv(path, lambda rows: rows[2].update(final_use="0"))
            return path.name

        def divergent_local_ledger(run):
            path = run / "observations.csv"
            shutil.copy2(cache / "observations.csv", path)
            rewrite_csv(path, lambda rows: rows[0].update(valid="0"))
            return None

        rejected_variant("orphan_audit", orphan_audit,
                         "factor-audit observation domain")
        rejected_variant("missing_audit", missing_audit,
                         "factor-audit observation domain")
        rejected_variant("duplicate_audit", duplicate_audit,
                         "duplicate observation in final factor audit")
        rejected_variant("wrong_classification", wrong_classification,
                         "final factor audit classification mismatch")
        rejected_variant("contradictory_expected_count",
                         contradictory_expected_count,
                         "final factor audit count or status mismatch")
        rejected_variant("contradictory_flags", contradictory_flags,
                         "noncandidate final mask flags are inconsistent")
        rejected_variant("divergent_local_ledger", divergent_local_ledger,
                         "local observation ledger does not match verified "
                         "parent cache")

        def rejected_parent_variant(name, recompute_parent_id, expected_error):
            variant_cache = root / f"{name}_cache"
            shutil.copytree(cache, variant_cache)
            ledger = variant_cache / "observations.csv"
            rewrite_csv(ledger, lambda rows: rows[0].update(valid="0"))
            manifest_path = variant_cache / "stage2_cache_manifest.json"
            manifest = json.loads(manifest_path.read_text())
            for payload in manifest["payloads"]:
                if payload["name"] == "observations.csv":
                    payload["sha256"] = sha(ledger)
            manifest["observation_mapping_sha256"] = sha(ledger)
            if recompute_parent_id:
                manifest["cache_id"] = scheduler.stage2_cache_identity(manifest)
            manifest_path.write_text(json.dumps(manifest))

            negative_batch = root / f"{name}_batch.json"
            negative_batch.write_text(json.dumps({
                "schema": "uifgo_t09_batch_manifest_v2", "status": "COMPLETE",
                "cells": [{"cell_id": name, "run_unit_id": "u",
                    "canonical_mode": "full_gate",
                    "execution_type": "FINAL_TRAJECTORY", "status": "COMPLETE",
                    "run_directory": str(final),
                    "cache_id": manifest["cache_id"],
                    "parent_cache_manifest": str(manifest_path),
                    "attempts": []}]}))
            output = root / f"{name}_evaluation.json"
            completed = subprocess.run([
                sys.executable, "-B", str(ROOT / "tools/paper/evaluate_runs.py"),
                "--batch-manifest", str(negative_batch),
                "--evaluation-manifest", str(evaluation), "--output", str(output)],
                text=True, capture_output=True)
            assert completed.returncode == 2, (name, completed.stderr)
            assert expected_error in completed.stderr, (name, completed.stderr)
            assert not output.exists(), name

        rejected_parent_variant(
            "stale_parent_cache_id", False,
            "cell parent cache content identity mismatch")
        rejected_parent_variant(
            "substituted_recomputed_parent", True,
            "final result is not bound to parent Stage-2 cache")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
