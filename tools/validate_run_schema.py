#!/usr/bin/env python3
"""Validate uwb-imu-pl raw run directories without third-party dependencies."""

import argparse
import csv
import hashlib
import json
import math
import pathlib
import re
import sys

# Complete 20-epoch hypothesis/evidence ID lists can exceed csv's 128 KiB default.
csv.field_size_limit(32 * 1024 * 1024)

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
        ",transaction_id,window_id,base_graph_version,linearization_version,selected_action_id,selected_action_type,fde_status,bridge_pl_x,bridge_pl_y,bridge_pl_z,history_provenance_valid,backend_updates,stale_state,controlled_reinitialization_required,historical_groups_removed,historical_groups_added,recovery_epoch_begin,recovery_epoch_end,reinitialization_request_id,reinitialization_phase,reinitialization_reason,reason",
    "transactions.csv": "timestamp_ns,transaction_id,window_id,base_graph_version,linearization_version,selected_action_id,fde_status,backend_updates,stale_state,reinitialization_required",
    "hypotheses.csv": "timestamp_ns,window_id,hypothesis_id,fault_unit_ids,prior_bound,p_md_allocation,hmi_allocation,monitorable,plausible,conditioned_statistic,log_evidence,reason",
    "candidates.csv": "timestamp_ns,window_id,action_id,action_type,cardinality,valid,post_detector_passed,covers_plausible_set,statistic,threshold,rank,dof,condition_number,hpl_m,vpl_m,selected,wall_ms,reason",
    "factor_ledger.csv": "factor_id,group_id,sensor,factor_kind,lifecycle,epoch_begin,epoch_end,time_begin_ns,time_end_ns,backend_slot,noise_model_id,model_id,health,source_ids,measurement_ids,fault_units,commit_graph_version,removed_graph_version,replacement_group_id,replaces_group_id,recovery_epoch",
    "health.csv": "timestamp_ns,source_id,sensor,previous_state,current_state,trigger,suspicion_count,shadow_pass_count,recovery_pass_count",
    "bridge.csv": "timestamp_ns,transaction_id,mode,consecutive_epochs,duration_s,integrity_model,calibration_id,bound_x,bound_y,bound_z,status",
    "fault_truth.csv": "timestamp_ns,sequence,anchor_id,fault_mode,active,outage,injected_bias_m,true_range_m,sensor_type,fault_kind,axis,epoch_begin,epoch_end,injected_value,injected_units",
})
V4_LEGACY_HEADERS = dict(V4_HEADERS)
V4_LEGACY_HEADERS.update({
    "integrity.csv": V2_HEADERS["integrity.csv"][:-len(",reason")] +
        ",transaction_id,window_id,base_graph_version,linearization_version,selected_action_id,selected_action_type,fde_status,bridge_pl_x,bridge_pl_y,bridge_pl_z,history_provenance_valid,backend_updates,stale_state,controlled_reinitialization_required,reason",
    "candidates.csv": "timestamp_ns,window_id,action_id,action_type,cardinality,valid,post_detector_passed,covers_plausible_set,statistic,threshold,rank,dof,condition_number,hpl_m,vpl_m,selected,reason",
    "factor_ledger.csv": "factor_id,group_id,sensor,factor_kind,lifecycle,epoch_begin,epoch_end,time_begin_ns,time_end_ns,backend_slot,noise_model_id,model_id,health",
})
V5_HEADERS = dict(V4_HEADERS)
V5_HEADERS.update({
    "hypotheses.csv": "timestamp_ns,window_id,hypothesis_id,fault_unit_ids,physical_source_ids,sensor,fault_kind,mode_ids,onset_epoch,onset_time_ns,parameter_dimension,fault_rank,sigma_min,sigma_max,condition_number,slope_x,slope_y,slope_z,boundary_direction_gram,noncentrality_boundary,prior_bound,p_md_allocation,hmi_allocation,monitorable,plausible,conditioned_statistic,log_evidence,reason",
    "candidates.csv": "timestamp_ns,window_id,action_id,action_type,physical_source_ids,removed_group_ids,added_group_ids,bridge_mode,cardinality,valid,post_detector_passed,covers_plausible_set,statistic,threshold,rank,dof,condition_number,information_logdet,risk_allocation,hpl_m,vpl_m,selected,evaluation_wall_ms,reason",
    "health.csv": "timestamp_ns,source_id,sensor,previous_state,current_state,trigger,evidence_statistic,evidence_threshold,plausible_hypothesis_ids,selected_action_id,suspicion_count,shadow_pass_count,recovery_pass_count,bridge_count,recovery_reset_count",
    "bridge.csv": "timestamp_ns,transaction_id,mode,consecutive_epochs,duration_s,model_id,dt_s,optimization_covariance_diagonal,integrity_model,calibration_id,bound_x,bound_y,bound_z,control_available,active,timeout,status",
})


# Run schema v5 keeps the same manifest/CSV layout while the *diagnostics*
# schema evolved: B1 (v12) appended the Z-response audit columns and B2 (v13)
# appended the coverage certificate columns to hypotheses.csv.  All three
# hypothesis headers are accepted so frozen baselines stay validatable.
V5_HYPOTHESES_B1 = V5_HEADERS["hypotheses.csv"][:-len(",reason")] + \
    ",z_rank,z_sigma_min,z_condition,z_classification,reason"
V5_HYPOTHESES_V13 = V5_HYPOTHESES_B1[:-len(",reason")] + \
    ",coverage_label,coverage_envelope_id,reason"
V5_HEADER_ALTERNATES = {"hypotheses.csv": (V5_HYPOTHESES_B1,
                                           V5_HYPOTHESES_V13)}


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


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return sum(1 for _ in csv.DictReader(stream))


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


def validate_v2(directory, manifest, expected_headers=V2_HEADERS,
                strict_timestamps=True, alternate_headers=None):
    required = {"states.csv", "integrity.csv", "events.csv", "ground_truth.csv",
                "fault_truth.csv", "summary.json", "resolved_config.yaml"}
    for name in required:
        if not (directory / name).is_file():
            fail(f"missing required v2 artifact: {name}")
    for name, expected in expected_headers.items():
        path = directory / name
        if not path.exists():
            continue
        allowed = (expected,) + tuple(
            (alternate_headers or {}).get(name, ()))
        if header(path) not in allowed:
            fail(f"{name}: header mismatch")
    validate_monotonic(directory / "states.csv", strict_timestamps)
    validate_monotonic(directory / "integrity.csv", strict_timestamps)
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
    selected_headers = V4_HEADERS
    if ((directory / "integrity.csv").is_file() and
            header(directory / "integrity.csv") ==
            V4_LEGACY_HEADERS["integrity.csv"]):
        selected_headers = V4_LEGACY_HEADERS
    validate_v2(directory, manifest, selected_headers)
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
    policy_fields = ("single_faults_enabled", "double_faults_enabled",
                     "supported_max_fault_cardinality",
                     "max_exclusion_cardinality")
    if any(field in scope for field in policy_fields):
        if not all(field in scope for field in policy_fields):
            fail("scope fault policy fields must be emitted together")
        single = scope["single_faults_enabled"]
        double = scope["double_faults_enabled"]
        supported = scope["supported_max_fault_cardinality"]
        monitored = scope.get("max_fault_cardinality")
        exclusion = scope["max_exclusion_cardinality"]
        if not isinstance(single, bool) or not isinstance(double, bool):
            fail("scope fault enable switches must be boolean")
        if not single and not double:
            fail("scope cannot disable every fault hypothesis cardinality")
        expected = 2 if double else 1
        if supported != 2 or monitored != expected:
            fail("scope supported/effective fault cardinality is inconsistent")
        if (not isinstance(exclusion, int) or isinstance(exclusion, bool) or
                exclusion < 1 or exclusion > supported):
            fail("scope max_exclusion_cardinality is invalid")
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


def id_set(value, field):
    if value == "":
        return set()
    try:
        result = {int(item) for item in value.split(";")}
    except ValueError as error:
        fail(f"{field}: malformed ID list")
    if any(item <= 0 for item in result):
        fail(f"{field}: IDs must be positive")
    return result


def validate_checksum_file(directory, relative):
    path = directory / relative
    if not path.is_file():
        fail(f"missing checksum file: {relative}")
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        if not match:
            fail(f"{relative}:{line_number}: malformed checksum row")
        artifact = pathlib.PurePosixPath(match.group(2))
        if artifact.is_absolute() or ".." in artifact.parts:
            fail(f"{relative}:{line_number}: unsafe checksum path")
        target = directory / pathlib.Path(artifact)
        if not target.is_file() or sha256(target) != match.group(1):
            fail(f"{relative}:{line_number}: checksum mismatch")


def validate_v5(directory, manifest):
    # Rejected attempts legitimately repeat the last committed state time.
    validate_v2(directory, manifest, V5_HEADERS, strict_timestamps=False,
                alternate_headers=V5_HEADER_ALTERNATES)
    for name in ("transactions.csv", "hypotheses.csv", "candidates.csv",
                 "factor_ledger.csv", "health.csv", "bridge.csv"):
        if not (directory / name).is_file():
            fail(f"missing required v5 artifact: {name}")
    resolved = (directory / "resolved_config.yaml").read_bytes()
    if fnv1a64(resolved) != manifest["config_hash"].lower():
        fail("resolved_config.yaml: content does not match config_hash")
    for field in ("protocol_sha256", "raw_inventory_sha256"):
        value = manifest.get(field)
        if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value):
            fail(f"run_manifest.json: {field} must be 64 lowercase hex digits")
    for field in ("protocol_path", "raw_inventory_path",
                  "artifact_checksum_path", "seed_domain",
                  "failure_catalog_path", "execution_command"):
        if not isinstance(manifest.get(field), str) or not manifest[field].strip():
            fail(f"run_manifest.json: {field} must be non-empty")
    if not isinstance(manifest.get("attempt"), int) or manifest["attempt"] < 1:
        fail("run_manifest.json: attempt must be a positive integer")
    protocol_path = manifest["protocol_path"]
    if protocol_path != "UNAVAILABLE":
        protocol = directory / protocol_path
        if not protocol.is_file() or sha256(protocol) != manifest["protocol_sha256"]:
            fail("protocol artifact is missing or does not match protocol_sha256")
    failure_catalog = manifest["failure_catalog_path"]
    if failure_catalog != "UNAVAILABLE" and not (directory / failure_catalog).is_file():
        fail("failure catalog artifact is missing")

    transactions = {}
    transaction_actions = {}
    timestamp_windows = {}
    with (directory / "transactions.csv").open(newline="", encoding="utf-8") as stream:
        for line, row in enumerate(csv.DictReader(stream), 2):
            transaction = int(row["transaction_id"])
            window = int(row["window_id"])
            if transaction <= 0 or window <= 0:
                fail(f"transactions.csv:{line}: zero transaction/window ID")
            if transaction in transactions:
                fail(f"transactions.csv:{line}: duplicate transaction ID")
            transactions[transaction] = window
            transaction_actions[window] = int(row["selected_action_id"])
            timestamp_windows[int(row["timestamp_ns"])] = window
            if int(row["backend_updates"]) not in (0, 1):
                fail(f"transactions.csv:{line}: backend_updates must be zero or one")

    windows = set(transactions.values())
    hypothesis_ids = set()
    with (directory / "hypotheses.csv").open(newline="", encoding="utf-8") as stream:
        for line, row in enumerate(csv.DictReader(stream), 2):
            if int(row["window_id"]) not in windows:
                fail(f"hypotheses.csv:{line}: dangling window_id")
            identifier = int(row["hypothesis_id"])
            key = (int(row["window_id"]), identifier)
            if identifier <= 0 or key in hypothesis_ids:
                fail(f"hypotheses.csv:{line}: invalid/duplicate hypothesis ID")
            hypothesis_ids.add(key)
            id_set(row["fault_unit_ids"], f"hypotheses.csv:{line}:fault_unit_ids")
            id_set(row["mode_ids"], f"hypotheses.csv:{line}:mode_ids")
            if not row["physical_source_ids"].strip():
                fail(f"hypotheses.csv:{line}: physical_source_ids must be non-empty")
            if int(row["parameter_dimension"]) <= 0:
                fail(f"hypotheses.csv:{line}: parameter_dimension must be positive")
            if row["reason"].strip() == "":
                fail(f"hypotheses.csv:{line}: reason must be non-empty")

    candidate_actions = {}
    selected_actions = {}
    with (directory / "candidates.csv").open(newline="", encoding="utf-8") as stream:
        for line, row in enumerate(csv.DictReader(stream), 2):
            window = int(row["window_id"])
            if window not in windows:
                fail(f"candidates.csv:{line}: dangling window_id")
            action = int(row["action_id"])
            key = (window, action)
            if action <= 0 or key in candidate_actions:
                fail(f"candidates.csv:{line}: invalid/duplicate action ID")
            candidate_actions[key] = row
            id_set(row["removed_group_ids"], f"candidates.csv:{line}:removed_group_ids")
            id_set(row["added_group_ids"], f"candidates.csv:{line}:added_group_ids")
            boolean(row["valid"], f"candidates.csv:{line}:valid")
            boolean(row["selected"], f"candidates.csv:{line}:selected")
            if row["selected"] in ("1", "true", "True"):
                if window in selected_actions:
                    fail(f"candidates.csv:{line}: multiple selected actions in window")
                selected_actions[window] = action
            if row["valid"] in ("0", "false", "False") and not row["reason"].strip():
                fail(f"candidates.csv:{line}: invalid candidate needs a reason")
            if float(row["evaluation_wall_ms"]) < 0:
                fail(f"candidates.csv:{line}: negative evaluation time")
    for window, action in transaction_actions.items():
        if action and selected_actions.get(window) != action:
            fail(f"transactions.csv: selected action {action} is absent for window {window}")

    factor_ids = set()
    group_ids = set()
    with (directory / "factor_ledger.csv").open(newline="", encoding="utf-8") as stream:
        for line, row in enumerate(csv.DictReader(stream), 2):
            factor = int(row["factor_id"])
            group = int(row["group_id"])
            if factor <= 0 or group <= 0:
                fail(f"factor_ledger.csv:{line}: invalid factor/group ID")
            factor_ids.add(factor)
            group_ids.add(group)
            id_set(row["measurement_ids"], f"factor_ledger.csv:{line}:measurement_ids")
            commit = int(row["commit_graph_version"])
            removed = int(row["removed_graph_version"])
            if removed and removed < commit:
                fail(f"factor_ledger.csv:{line}: removed version precedes commit")
            replacement = int(row["replacement_group_id"])
            replaces = int(row["replaces_group_id"])
            if replacement and replaces:
                fail(f"factor_ledger.csv:{line}: both replacement directions are set")

    frozen_groups = {}
    frozen_candidate_rows = []
    diagnostic_attempts = directory / "diagnostic_attempts.csv"
    if diagnostic_attempts.exists():
        with diagnostic_attempts.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                if row["schema_version"].endswith(
                        ("/v3", "/v4", "/v5", "/v6", "/v7", "/v8", "/v9", "/v10", "/v11", "/v12", "/v13", "/v14")):
                    frozen_groups[int(row["input_attempt_id"])] = id_set(row["frozen_group_ids"], "frozen_group_ids")
        if frozen_groups:
            with (directory / "diagnostic_candidates.csv").open(newline="", encoding="utf-8") as stream:
                frozen_candidate_rows = [frozen_groups.get(int(row["input_attempt_id"]))
                                         for row in csv.DictReader(stream)]
    with (directory / "candidates.csv").open(newline="", encoding="utf-8") as stream:
        for line, row in enumerate(csv.DictReader(stream), 2):
            referenced = id_set(row["removed_group_ids"], "removed_group_ids")
            # Pending factors can be present in the frozen window and then
            # discarded without ever entering the committed factor ledger.
            actual = (frozen_candidate_rows[line - 2] if frozen_candidate_rows else group_ids)
            if actual is None: actual = group_ids
            if not referenced.issubset(actual):
                fail(f"candidates.csv:{line}: removed group missing from frozen window/ledger")

    with (directory / "health.csv").open(newline="", encoding="utf-8") as stream:
        for line, row in enumerate(csv.DictReader(stream), 2):
            timestamp = int(row["timestamp_ns"])
            window = timestamp_windows.get(timestamp)
            for hypothesis in id_set(row["plausible_hypothesis_ids"],
                                     f"health.csv:{line}:plausible_hypothesis_ids"):
                if window is None or (window, hypothesis) not in hypothesis_ids:
                    fail(f"health.csv:{line}: dangling plausible hypothesis ID")

    inventory_path = manifest["raw_inventory_path"]
    if inventory_path != "UNAVAILABLE":
        inventory = directory / inventory_path
        if not inventory.is_file() or sha256(inventory) != manifest["raw_inventory_sha256"]:
            fail("raw inventory is missing or checksum does not match")
        payload = json.loads(inventory.read_text(encoding="utf-8"))
        entries = payload.get("artifacts")
        if not isinstance(entries, list):
            fail("raw inventory artifacts must be a list")
        for entry in entries:
            relative_artifact = pathlib.PurePosixPath(entry["path"])
            if relative_artifact.is_absolute() or ".." in relative_artifact.parts:
                fail(f"raw inventory unsafe path: {entry.get('path')}")
            artifact = directory / pathlib.Path(relative_artifact)
            if not artifact.is_file() or sha256(artifact) != entry["sha256"]:
                fail(f"raw inventory mismatch: {entry.get('path')}")
            if artifact.suffix == ".csv" and rows(artifact) != entry["rows"]:
                fail(f"raw inventory row-count mismatch: {entry['path']}")
    checksum_path = manifest["artifact_checksum_path"]
    if checksum_path != "UNAVAILABLE":
        validate_checksum_file(directory, checksum_path)


def validate(directory):
    diagnostic_path = directory / "diagnostic_attempts.csv"
    if diagnostic_path.exists():
        from gate_d_diagnostics import read_rows, validate_attachments
        if read_rows(directory, "diagnostic_attempts.csv"):
            # Core timing linkage is required by the performance runner; other
            # RunLogger clients may write their timing through another facade.
            validate_attachments(directory, require_legacy_timing=False)
    manifest_path = directory / "run_manifest.json"
    if not manifest_path.is_file():
        fail("missing run_manifest.json")
    with manifest_path.open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    version = manifest.get("schema_version")
    if version not in ("uwb-imu-pl/v1", "uwb-imu-pl/v2", "uwb-imu-pl/v3",
                       "uwb-imu-pl/v4", "uwb-imu-pl/v5"):
        fail(f"unsupported schema_version: {version!r}")
    for field, kind in (("git_sha", str), ("config_hash", str),
                        ("seed", int), ("git_dirty", bool)):
        if not isinstance(manifest.get(field), kind):
            fail(f"run_manifest.json: invalid {field} type")
    if not manifest["git_sha"]:
        fail("run_manifest.json: empty git_sha")
    if not re.fullmatch(r"[0-9a-fA-F]{16}", manifest["config_hash"]):
        fail("run_manifest.json: config_hash is not 16 hexadecimal digits")
    if version == "uwb-imu-pl/v5":
        validate_v5(directory, manifest)
    elif version == "uwb-imu-pl/v4":
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
