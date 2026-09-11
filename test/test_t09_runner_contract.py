#!/usr/bin/env python3
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]


MOCK = r'''#!/usr/bin/env python3
import hashlib, json, os, pathlib, struct, sys, yaml
args = sys.argv[1:]
def value(flag): return args[args.index(flag)+1]
root = pathlib.Path(value("--output-root")); run_id = value("--run-id")
method = value("--method") if "--method" in args else "structured_debias"
run = root / run_id; run.mkdir()
(run / "common_preparation.json").write_text(json.dumps({"schema":"uifgo_t09_common_preparation_v1","common_preparation_id":"t09common-sha256:mock"}))
(run / "observations.csv").write_text("obs_id,valid,planned\n1,1,1\n2,1,1\n")
(run / "trajectory.tum").write_text("0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n2 2 0 0 0 0 0 1\n")
if method == "structured_debias":
    (run / "partition.json").write_text("{}\n")
    (run / "support_snapshot.csv").write_text("obs_id,bias_m\n1,0.1\n")
    (run / "segments.csv").write_text("segment_id,segment_ordinal,obs_count\ns,0,1\n")
    (run / "factor_metadata.csv").write_text("factor_index,obs_id,factor_type\n0,1,uwb_segment_range\n")
    (run / "refit_iterations.csv").write_text("outer_iteration\n0\n")
    (run / "scores_decision.csv").write_text("group_id,status,eligible,valid_score_exported,score_availability,eta,s_m,s_is_infinite,linearization_id\ng,INELIGIBLE_SHORT_OR_BOUNDARY_DEBUG,0,0,NOT_APPLICABLE_SHORT_OR_BOUNDARY,,,,lin\n")
    (run / "segment_fit_scores.csv").write_text("group_id,segment_id,segment_ordinal,gamma,short_support_debug,boundary,status,linearization_id\ng,s,0,1.0,1,0,INELIGIBLE_SHORT_OR_BOUNDARY_DEBUG,lin\n")
    (run / "groups.csv").write_text("group_id,start_time,end_time,segment_ordinals,eligible,status\ng,0,1,0,0,SHORT\n")
    (run / "imu_bias.csv").write_text("keyframe_id,bax,bay,baz,bgx,bgy,bgz\n0,0,0,0,0,0,0\n")
    (run / "stage2_values.csv").write_text("key,type,v0\nx0,POSE3,0\n")
    (run / "stage2_content_identity.json").write_text(json.dumps({"values_sha256":"sha256:values","graph_linearization_sha256":"sha256:graph"}))
    (run / "stage2_producer_context.json").write_text(json.dumps({
        "schema":"uifgo_t09_stage2_producer_context_v1",
        "stage1_config_sha256":"sha256:stage1",
        "stage2_refit_config_sha256":"sha256:stage2",
        "stage3_score_config_sha256":"sha256:stage3"}))
    (run / "input_manifest.json").write_text(json.dumps({"source_hash_sha256":"sha256:x"}))
    (run / "capability_status.json").write_text(json.dumps({"segment_refit":"CONVERGED","discovery":"CONVERGED_STAGE1","recoverability_score":"ONE_OR_MORE_GROUP_SCORES_UNAVAILABLE"}))
    (run / "run_status.json").write_text(json.dumps({"status":"PARTIAL_STAGE2_OK_SCORE_UNAVAILABLE","exit_code":1,"reason":"partial"}))
    sys.exit(1)
(run / "baseline_factor_audit.csv").write_text("obs_id,final_use,reason\n1,1,RETAINED\n2,1,RETAINED\n")
if "--execution-type" in args and value("--execution-type") == "FINAL_TRAJECTORY":
    inference = "t08inference-sha256:mock"
    cache = json.loads(pathlib.Path(value("--stage2-cache-manifest")).read_text())
    point = value("--operating-point-id")
    cfg = yaml.safe_load(pathlib.Path(value("--config")).read_text())
    nlos = cfg["nlos"]
    threshold_text = "uifgo-t09-thresholds-binary64-v1\n" + "".join(
        "%s=%s\n" % (name, struct.pack(">d", float(nlos.get(name, 0.0))).hex())
        for name in ("tau_eta", "tau_s_m", "tau_gamma"))
    threshold_hash = "sha256:" + hashlib.sha256(threshold_text.encode()).hexdigest()
    policy = "T09_%s_FINAL_AUDIT_V1" % method.upper()
    fields = [cache["cache_id"], method, policy, threshold_hash,
              "T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION",
              "sha256:final-config", "sha256:solver",
              "t09common-sha256:mock"]
    encoded = bytearray(b"uifgo-t09-final-request-v1\n")
    for field in fields:
        raw = field.encode(); encoded.extend(str(len(raw)).encode() + b":" + raw + b"\n")
    request = "t09finalrequest-sha256:" + hashlib.sha256(encoded).hexdigest()
    (run / "final_masks.csv").write_text("inference_id,obs_id,candidate,decision_use,final_use\n%s,1,1,1,1\n" % inference)
    (run / "final_factor_audit.csv").write_text("inference_id,obs_id,final_factor_count,ok\n%s,1,1,1\n" % inference)
    (run / "final_content_identity.json").write_text(json.dumps({"inference_id":inference}))
    (run / "final_inference_summary.json").write_text(json.dumps({
        "inference_id":inference,"valid_estimate":True,"factor_audit_status":"OK",
        "tau_eta":float(nlos.get("tau_eta",0.0)),
        "tau_s_m":float(nlos.get("tau_s_m",0.0)),
        "tau_gamma":float(nlos.get("tau_gamma",0.0))}))
    (run / "run_manifest.json").write_text(json.dumps({
        "canonical_mode":method,"inference_id":inference,
        "operating_point_id":point,"stage2_cache_id":cache["cache_id"],
        "final_request_id":request,"policy_version":policy,
        "thresholds_sha256":threshold_hash,
        "threshold_provenance":"T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION",
        "final_refit_score_config_sha256":"sha256:final-config",
        "solver_sha256":"sha256:solver",
        "common_preparation_id":"t09common-sha256:mock",
        "export_verification_status":"VERIFIED"}))
    (run / "run_status.json").write_text(json.dumps({
        "status":"OK","exit_code":0,"valid_estimate_exported":True,
        "inference_id":inference,"final_request_id":request,
        "method_argument":method,
        "eta_synthetic_final":os.environ.get("UIFGO_T09_ETA_ONLY_SYNTHETIC_FINAL")}))
else:
    (run / "run_status.json").write_text(json.dumps({
        "status":"OK","exit_code":0,"valid_estimate_exported":True,
        "method_argument":method,
        "eta_synthetic_final":os.environ.get("UIFGO_T09_ETA_ONLY_SYNTHETIC_FINAL")}))
'''


def main():
    scheduler_spec = importlib.util.spec_from_file_location(
        "t09_scheduler", ROOT / "tools/paper/run_experiments.py")
    scheduler = importlib.util.module_from_spec(scheduler_spec)
    scheduler_spec.loader.exec_module(scheduler)
    vector = {
        "cache_namespace": "AUTO_DISCOVERY",
        "debug_label": "RQ3_AUTO_DISCOVERY_END_TO_END",
        "common_preparation_id": "t09common-sha256:a",
        "source_identity": "sha256:b",
        "producer_common_config_sha256": "t09commonconfig-sha256:c0",
        "producer_stage2_config_sha256": "t09stage2config-sha256:c1",
        "stage1_config_sha256": "sha256:stage1",
        "stage2_refit_config_sha256": "sha256:stage2",
        "stage3_score_config_sha256": "sha256:stage3",
        "support_partition_sha256": "sha256:c",
        "observation_mapping_sha256": "sha256:d",
        "stage2_graph_linearization_sha256": "sha256:e",
        "stage2_values_sha256": "sha256:f",
        "factor_metadata_sha256": "sha256:g",
        "stage2_trace_sha256": "sha256:h",
        "score_table_sha256": "sha256:i",
        "producer_commit": "deadbeef",
        "producer_binary_sha256": "sha256:binary",
        "producer_abi_sha256": "t09abi-sha256:abi",
        "producer_toolchain_sha256": "t09toolchain-sha256:toolchain",
        "stage2_status": "CONVERGED",
        "score_status": "COMPLETE_WITH_SCORE_UNAVAILABLE",
        "payloads": [{
            "name": "scores.csv",
            "sha256": "sha256:" + hashlib.sha256(
                b"group,status\ng,OK\n").hexdigest(),
        }],
    }
    assert scheduler.stage2_cache_identity(vector) == (
        "t09stage2cache-sha256:"
        "ac94b180a84c4d66cc2200d67139f1a557036523763c07624668e837c15a78d5")
    with tempfile.TemporaryDirectory(prefix="uifgo-t09-runner-") as temporary:
        root = Path(temporary)
        mock = root / "mock_runner.py"
        mock.write_text(MOCK)
        mock.chmod(0o755)
        source = root / "source.yaml"
        source.write_text(yaml.safe_dump({
            "dataset": {"interface": "original"},
            "topics": {"imu": "/imu", "uwb": "/uwb", "gt_odom": "/gt"},
            "nlos": {"mode": "automatic_discovery", "score_recoverability": True},
        }))
        manifest = root / "batch.yaml"
        manifest.write_text(yaml.safe_dump({
            "schema": "uifgo_t09_batch_v1", "role": "development",
            "parameter_provenance": "T09_DEVELOPMENT_ENGINEERING_TEST_ONLY",
            "run_units": [{
                "run_unit_id": "recording/base/7/full", "recording_id": "recording",
                "base_trajectory_id": "base", "seed": 7, "prefix_identity": "full",
                "config": "source.yaml", "cells": [
                    {"mode": "structured_debias", "execution_type": "AUTOMATIC_STAGE2_TRAJECTORY"},
                    {"mode": "fit_only", "execution_type": "CACHE_DIAGNOSTIC",
                     "thresholds": {"tau_gamma": 1.0}},
                    {"mode": "all_range", "execution_type": "BASELINE_TRAJECTORY"},
                ],
            }],
        }))
        output = root / "output"
        completed = subprocess.run([
            sys.executable, "-B", str(ROOT / "tools/paper/run_experiments.py"),
            "--manifest", str(manifest), "--runner", str(mock),
            "--output-root", str(output)], text=True, capture_output=True)
        assert completed.returncode == 0, completed.stderr
        batch = json.loads((output / "batch_manifest.json").read_text())
        assert batch["status"] == "COMPLETE"
        assert batch["summary"]["terminal_cells"] == 3
        cells = {cell["canonical_mode"]: cell for cell in batch["cells"]}
        assert cells["all_range"]["status"] == "COMPLETE"
        assert cells["structured_debias"]["cache_status"] == "COMPLETE_WITH_SCORE_UNAVAILABLE"
        assert cells["structured_debias"]["status"] == "COMPLETE_WITH_SCORE_UNAVAILABLE"
        assert cells["fit_only"]["status"] == "COMPLETE"
        assert cells["all_range"]["common_preparation_id"] == cells["fit_only"]["common_preparation_id"]
        effective = yaml.safe_load(next((output / "effective_configs").glob("*.yaml")).read_text())
        assert "gt_odom" not in effective.get("topics", {})
        cache_manifest = next((output / "caches").glob("*/stage2_cache_manifest.json"))
        cache = json.loads(cache_manifest.read_text())
        forbidden = {"accepted_mask", "fallback", "final_request_id", "inference_id", "final_graph"}
        assert not forbidden.intersection(cache)
        diagnostic = json.loads((Path(cells["fit_only"]["run_directory"]) / "diagnostic.json").read_text())
        assert diagnostic["candidate_use_coverage"]["value"] == 0.0
        assert diagnostic["eligible_use_coverage"]["status"] == "UNDEFINED_ZERO_DENOMINATOR"

        eta_manifest = root / "eta_batch.yaml"
        eta_document = {
            "schema": "uifgo_t09_batch_v1", "role": "development",
            "parameter_provenance": "T09_SYNTHETIC_FINAL_TEST_ONLY",
            "run_units": [{
                "run_unit_id": "synthetic/base/1/full", "recording_id": "synthetic",
                "base_trajectory_id": "base", "seed": 1, "prefix_identity": "full",
                "config": "source.yaml", "cells": [
                    {"mode": "structured_debias",
                     "execution_type": "CACHE_PRODUCER",
                     "cache_namespace": "AUTO_DISCOVERY"},
                    {"mode": "eta_only", "execution_type": "FINAL_TRAJECTORY",
                     "thresholds": {"tau_eta": 1.0}},
                ],
            }],
        }
        eta_manifest.write_text(yaml.safe_dump(eta_document))
        rejected = subprocess.run([
            sys.executable, "-B", str(ROOT / "tools/paper/run_experiments.py"),
            "--manifest", str(eta_manifest), "--runner", str(mock),
            "--output-root", str(root / "eta_rejected")], text=True,
            capture_output=True)
        assert rejected.returncode == 2
        assert "predeclared synthetic_final" in rejected.stderr

        eta_document["run_units"][0]["cells"][1]["synthetic_final"] = True
        eta_manifest.write_text(yaml.safe_dump(eta_document))
        eta_output = root / "eta_output"
        accepted = subprocess.run([
            sys.executable, "-B", str(ROOT / "tools/paper/run_experiments.py"),
            "--manifest", str(eta_manifest), "--runner", str(mock),
            "--output-root", str(eta_output)], text=True, capture_output=True)
        assert accepted.returncode == 0, accepted.stderr
        eta_batch = json.loads((eta_output / "batch_manifest.json").read_text())
        eta_cell = next(cell for cell in eta_batch["cells"]
                        if cell["canonical_mode"] == "eta_only")
        eta_status = json.loads(
            (Path(eta_cell["run_directory"]) / "run_status.json").read_text())
        assert eta_status["method_argument"] == "eta_only"
        assert eta_status["eta_synthetic_final"] == "1"
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
