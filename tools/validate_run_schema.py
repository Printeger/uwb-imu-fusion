#!/usr/bin/env python3
"""Validate uwb-imu-pl raw run directories without third-party dependencies."""

import argparse
import csv
import json
import math
import pathlib
import re
import sys

V1_INTEGRITY = "timestamp_ns,detector,statistic,threshold,dof,passed,global_graph_statistic,uwb_postfit_statistic,conditional_statistic,pl_x,pl_y,pl_z,hpl_m,vpl,availability,label,formal_eligible,risk_budget_valid,allocated_hmi_risk,hmi_risk_requirement,batch_committed,reason"
V2_HEADERS = {
    "states.csv": "timestamp_ns,state_id,px,py,pz,qw,qx,qy,qz,vx,vy,vz,bax,bay,baz,bgx,bgy,bgz",
    "residuals.csv": "timestamp_ns,factor_id,anchor_id,row_role,raw,whitened",
    "integrity.csv": "timestamp_ns,group_size,measurement_model_valid,global_statistic,global_threshold,global_dof,global_passed,postfit_statistic,postfit_threshold,postfit_dof,postfit_passed,conditional_statistic,conditional_threshold,conditional_dof,conditional_passed,conditional_formal,pl_x,pl_y,pl_z,hpl_m,vpl_m,availability,label,formal_eligible,risk_budget_valid,allocated_hmi_risk,hmi_risk_requirement,batch_committed,reason",
    "timing.csv": "timestamp_ns,epoch,stage,wall_ms,problem_size,hypothesis_count,factor_count,cold_warm,success",
    "events.csv": "timestamp_ns,sequence,event,detail",
    "ground_truth.csv": "timestamp_ns,px,py,pz,qw,qx,qy,qz",
    "fault_truth.csv": "timestamp_ns,sequence,anchor_id,fault_mode,active,outage,injected_bias_m,true_range_m",
}
V4_HEADERS = dict(V2_HEADERS)
V4_HEADERS.update({
    "integrity.csv": V2_HEADERS["integrity.csv"][:-len(",reason")] +
        ",transaction_id,window_id,base_graph_version,linearization_version,selected_action_id,selected_action_type,fde_status,bridge_pl_x,bridge_pl_y,bridge_pl_z,history_provenance_valid,backend_updates,stale_state,controlled_reinitialization_required,reason",
    "transactions.csv": "timestamp_ns,transaction_id,window_id,base_graph_version,linearization_version,selected_action_id,fde_status,backend_updates,stale_state,reinitialization_required",
    "hypotheses.csv": "timestamp_ns,window_id,hypothesis_id,fault_unit_ids,prior_bound,p_md_allocation,hmi_allocation,monitorable,plausible,conditioned_statistic,log_evidence,reason",
    "candidates.csv": "timestamp_ns,window_id,action_id,action_type,cardinality,valid,post_detector_passed,covers_plausible_set,statistic,threshold,rank,dof,condition_number,hpl_m,vpl_m,selected,reason",
    "factor_ledger.csv": "factor_id,group_id,sensor,factor_kind,lifecycle,epoch_begin,epoch_end,time_begin_ns,time_end_ns,backend_slot,noise_model_id,model_id,health",
    "health.csv": "timestamp_ns,source_id,sensor,previous_state,current_state,trigger,suspicion_count,shadow_pass_count,recovery_pass_count",
    "bridge.csv": "timestamp_ns,transaction_id,mode,consecutive_epochs,duration_s,integrity_model,calibration_id,bound_x,bound_y,bound_z,status",
    "fault_truth.csv": "timestamp_ns,sequence,anchor_id,fault_mode,active,outage,injected_bias_m,true_range_m,sensor_type,fault_kind,axis,epoch_begin,epoch_end,injected_value,injected_units",
})


def fail(message):
    raise ValueError(message)


def header(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return stream.readline().rstrip("\r\n")


def boolean(value, field):
    if value not in ("0", "1", "true", "false", "True", "False"):
        fail(f"{field}: invalid boolean {value!r}")


def fnv1a64(payload):
    value = 1469598103934665603
    for byte in payload:
        value ^= byte
        value = (value * 1099511628211) & 0xffffffffffffffff
    return f"{value:016x}"


def validate_monotonic(path, strict):
    previous = None
    with path.open(newline="", encoding="utf-8") as stream:
        for line, row in enumerate(csv.DictReader(stream), 2):
            try:
                current = int(row["timestamp_ns"])
            except (KeyError, ValueError):
                fail(f"{path}:{line}: invalid timestamp_ns")
            if previous is not None and (current <= previous if strict else current < previous):
                fail(f"{path}:{line}: timestamp rollback")
            previous = current


def validate_v2(directory, manifest, expected_headers=V2_HEADERS):
    required = {"states.csv", "integrity.csv", "events.csv", "ground_truth.csv",
                "fault_truth.csv", "summary.json", "resolved_config.yaml"}
    for name in required:
        if not (directory / name).is_file():
            fail(f"missing required v2 artifact: {name}")
    for name, expected in expected_headers.items():
        path = directory / name
        if path.exists() and header(path) != expected:
            fail(f"{name}: header mismatch")
    validate_monotonic(directory / "states.csv", True)
    validate_monotonic(directory / "integrity.csv", True)
    validate_monotonic(directory / "events.csv", False)

    with (directory / "events.csv").open(newline="", encoding="utf-8") as stream:
        previous_sequence = None
        for line, row in enumerate(csv.DictReader(stream), 2):
            sequence = int(row["sequence"])
            if previous_sequence is not None and sequence <= previous_sequence:
                fail(f"events.csv:{line}: sequence is not strictly increasing")
            previous_sequence = sequence

    with (directory / "integrity.csv").open(newline="", encoding="utf-8") as stream:
        for line, row in enumerate(csv.DictReader(stream), 2):
            for field in ("measurement_model_valid", "global_passed",
                          "postfit_passed", "conditional_passed",
                          "conditional_formal", "formal_eligible",
                          "risk_budget_valid", "batch_committed"):
                boolean(row[field], f"integrity.csv:{line}:{field}")
            if int(row["group_size"]) < 0:
                fail(f"integrity.csv:{line}: negative group_size")
            formal = row["conditional_formal"] in ("1", "true", "True")
            if formal:
                for field in ("conditional_statistic", "conditional_threshold"):
                    if not math.isfinite(float(row[field])):
                        fail(f"integrity.csv:{line}:{field}: formal value not finite")

    with (directory / "summary.json").open(encoding="utf-8") as stream:
        summary = json.load(stream)
    for field in ("processed", "committed", "rejected", "errors"):
        if not isinstance(summary.get(field), int) or summary[field] < 0:
            fail(f"summary.json: {field} must be a non-negative integer")
    if not isinstance(summary.get("metrics"), dict):
        fail("summary.json: metrics must be an object")


def validate_v3(directory, manifest):
    validate_v2(directory, manifest)
    fixed_lag = manifest.get("fixed_lag_epochs")
    if not isinstance(fixed_lag, int) or isinstance(fixed_lag, bool):
        fail("run_manifest.json: fixed_lag_epochs must be an integer")
    if fixed_lag != 0 and fixed_lag < 2:
        fail("run_manifest.json: fixed_lag_epochs must be 0 or at least 2")
    command = manifest.get("execution_command")
    if not isinstance(command, str) or not command.strip():
        fail("run_manifest.json: execution_command must be non-empty")

    resolved_path = directory / "resolved_config.yaml"
    payload = resolved_path.read_bytes()
    if fnv1a64(payload) != manifest["config_hash"].lower():
        fail("resolved_config.yaml: content does not match config_hash")
    match = re.search(rb"(?m)^\s*fixed_lag_epochs:\s*([0-9]+)\s*$",
                      payload)
    if match is None or int(match.group(1)) != fixed_lag:
        fail("resolved_config.yaml: fixed_lag_epochs does not match manifest")


def validate_v4(directory, manifest):
    validate_v2(directory, manifest, V4_HEADERS)
    for name in ("transactions.csv", "hypotheses.csv", "candidates.csv",
                 "factor_ledger.csv", "health.csv", "bridge.csv"):
        if not (directory / name).is_file():
            fail(f"missing required v4 artifact: {name}")
    fixed_lag = manifest.get("fixed_lag_epochs")
    if not isinstance(fixed_lag, int) or isinstance(fixed_lag, bool):
        fail("run_manifest.json: fixed_lag_epochs must be an integer")
    resolved = (directory / "resolved_config.yaml").read_bytes()
    if fnv1a64(resolved) != manifest["config_hash"].lower():
        fail("resolved_config.yaml: content does not match config_hash")
    scope = manifest.get("scope")
    evidence = manifest.get("gate_j_evidence")
    if not isinstance(scope, dict) or not isinstance(evidence, dict):
        fail("v4 manifest requires machine-readable scope and gate_j_evidence")
    formal = manifest.get("formal_eligible")
    if not isinstance(formal, bool):
        fail("v4 manifest formal_eligible must be boolean")
    evidence_complete = (evidence.get("gates_a_to_i_complete") is True and
                         evidence.get("independent_review_complete") is True and
                         all(isinstance(evidence.get(key), str) and
                             evidence[key].strip() for key in
                             ("risk_calibration_id",
                              "noise_overbound_calibration_id",
                              "bridge_calibration_id")))
    if formal and not evidence_complete:
        fail("formal_eligible cannot be true without complete Gate J evidence")


def validate(directory):
    manifest_path = directory / "run_manifest.json"
    if not manifest_path.is_file():
        fail("missing run_manifest.json")
    with manifest_path.open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    version = manifest.get("schema_version")
    if version not in ("uwb-imu-pl/v1", "uwb-imu-pl/v2", "uwb-imu-pl/v3",
                       "uwb-imu-pl/v4"):
        fail(f"unsupported schema_version: {version!r}")
    for field, kind in (("git_sha", str), ("config_hash", str),
                        ("seed", int), ("git_dirty", bool)):
        if not isinstance(manifest.get(field), kind):
            fail(f"run_manifest.json: invalid {field} type")
    if not manifest["git_sha"]:
        fail("run_manifest.json: empty git_sha")
    if not re.fullmatch(r"[0-9a-fA-F]{16}", manifest["config_hash"]):
        fail("run_manifest.json: config_hash is not 16 hexadecimal digits")
    if version == "uwb-imu-pl/v4":
        validate_v4(directory, manifest)
    elif version == "uwb-imu-pl/v3":
        validate_v3(directory, manifest)
    elif version == "uwb-imu-pl/v2":
        validate_v2(directory, manifest)
    else:
        path = directory / "integrity.csv"
        if path.exists() and header(path) != V1_INTEGRITY:
            fail("integrity.csv: unsupported v1 header")
    return version


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=pathlib.Path)
    args = parser.parse_args()
    try:
        version = validate(args.run_directory)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print(f"PASS: {version} {args.run_directory}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
