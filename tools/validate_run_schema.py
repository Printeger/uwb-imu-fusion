#!/usr/bin/env python3
"""Validate uwb-imu-pl/v1 and v2 raw run directories without dependencies."""

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


def fail(message):
    raise ValueError(message)


def header(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return stream.readline().rstrip("\r\n")


def boolean(value, field):
    if value not in ("0", "1", "true", "false", "True", "False"):
        fail(f"{field}: invalid boolean {value!r}")


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


def validate_v2(directory, manifest):
    required = {"states.csv", "integrity.csv", "events.csv", "ground_truth.csv",
                "fault_truth.csv", "summary.json", "resolved_config.yaml"}
    for name in required:
        if not (directory / name).is_file():
            fail(f"missing required v2 artifact: {name}")
    for name, expected in V2_HEADERS.items():
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


def validate(directory):
    manifest_path = directory / "run_manifest.json"
    if not manifest_path.is_file():
        fail("missing run_manifest.json")
    with manifest_path.open(encoding="utf-8") as stream:
        manifest = json.load(stream)
    version = manifest.get("schema_version")
    if version not in ("uwb-imu-pl/v1", "uwb-imu-pl/v2"):
        fail(f"unsupported schema_version: {version!r}")
    for field, kind in (("git_sha", str), ("config_hash", str),
                        ("seed", int), ("git_dirty", bool)):
        if not isinstance(manifest.get(field), kind):
            fail(f"run_manifest.json: invalid {field} type")
    if not manifest["git_sha"]:
        fail("run_manifest.json: empty git_sha")
    if not re.fullmatch(r"[0-9a-fA-F]{16}", manifest["config_hash"]):
        fail("run_manifest.json: config_hash is not 16 hexadecimal digits")
    if version == "uwb-imu-pl/v2":
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
