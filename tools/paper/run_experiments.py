#!/usr/bin/env python3
"""T09 deterministic batch scheduler.

The scheduler preregisters run-unit × method cells, keeps every process in a
unique directory, and distinguishes estimator failures from infrastructure
failures.  It never receives an evaluation manifest or truth path.
"""

from __future__ import annotations

import argparse
import copy
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import time
import uuid

import yaml


CANONICAL_MODES = {
    "lcb_partial", "lcb_fixed_full", "suppress_all",
    "all_range", "robust_huber", "robust_cauchy", "fixed_rejection",
    "structured_bias_only", "structured_debias", "fit_only", "s_fit",
    "full_gate", "eta_only", "nominal_curvature", "oracle_reference",
}
BASELINES = {"all_range", "robust_huber", "robust_cauchy", "fixed_rejection"}
DIAGNOSTICS = {"fit_only", "s_fit", "full_gate", "eta_only", "nominal_curvature"}
FINAL_TYPES = {"BASELINE_TRAJECTORY", "STAGE1_TRAJECTORY",
               "AUTOMATIC_STAGE2_TRAJECTORY", "FINAL_TRAJECTORY"}
EXECUTION_TYPES = FINAL_TYPES | {"CACHE_PRODUCER", "CACHE_DIAGNOSTIC",
                                "EVALUATION_REFERENCE"}
STAGE2_REPLAY_REQUIRED_PAYLOADS = {
    "input_manifest.json", "observations.csv", "partition.json",
    "segments.csv", "factor_metadata.csv", "refit_iterations.csv",
    "scores_decision.csv", "segment_fit_scores.csv", "groups.csv",
    "stage2_values.csv", "stage2_content_identity.json",
    "stage2_producer_context.json",
}

POLICY_THRESHOLD_KEYS = {
    "fit_only": {"tau_gamma"},
    "s_fit": {"tau_s_m", "tau_gamma"},
    "full_gate": {"tau_eta", "tau_s_m", "tau_gamma"},
    "eta_only": {"tau_eta"},
    "nominal_curvature": {"tau_N_m2_inv"},
}

FINAL_ONLY_NLOS_KEYS = {
    "tau_eta", "tau_s_m", "tau_gamma", "gate_parameter_provenance",
    "final_inference_enabled",
}


def infer_path(cell: dict) -> str:
    if cell.get("path"):
        return str(cell["path"])
    if cell.get("cache_namespace") == "FIXED_PARTITION_DEBUG":
        return "FIXED_PARTITION_DEBUG"
    if cell.get("mode") == "oracle_reference":
        return "EVALUATION_ONLY"
    if cell.get("mode") in BASELINES:
        return "DIRECT_COMMON_PREPARATION"
    return "AUTO_DISCOVERY"


def validate_mode_execution_path(cell: dict) -> None:
    mode = cell["mode"]
    execution = cell["execution_type"]
    path = infer_path(cell)
    if (mode == "eta_only" and execution == "FINAL_TRAJECTORY" and
            cell.get("synthetic_final") is not True):
        raise ValueError(
            "eta_only final trajectory must be explicitly predeclared synthetic_final")
    allowed = False
    if path == "DIRECT_COMMON_PREPARATION":
        allowed = mode in BASELINES and execution == "BASELINE_TRAJECTORY"
    elif path == "AUTO_DISCOVERY":
        allowed = ((mode == "structured_bias_only" and
                    execution == "STAGE1_TRAJECTORY") or
                   (mode == "structured_debias" and execution in {
                       "CACHE_PRODUCER", "AUTOMATIC_STAGE2_TRAJECTORY"}) or
                   (mode in DIAGNOSTICS and execution == "CACHE_DIAGNOSTIC") or
                   (mode in {"fit_only", "s_fit", "full_gate", "lcb_partial", "lcb_fixed_full", "suppress_all", "structured_debias"} and
                    execution == "FINAL_TRAJECTORY") or
                   (mode == "eta_only" and execution == "FINAL_TRAJECTORY" and
                    cell.get("synthetic_final") is True))
    elif path == "FIXED_PARTITION_DEBUG":
        allowed = ((mode == "structured_debias" and
                    execution == "CACHE_PRODUCER") or
                   (mode in DIAGNOSTICS and execution == "CACHE_DIAGNOSTIC") or
                   (mode in {"fit_only", "s_fit", "full_gate", "lcb_partial", "lcb_fixed_full", "suppress_all", "structured_debias"} and
                    execution == "FINAL_TRAJECTORY") or
                   (mode == "eta_only" and execution == "FINAL_TRAJECTORY" and
                    cell.get("synthetic_final") is True))
    elif path == "EVALUATION_ONLY":
        allowed = mode == "oracle_reference" and execution == "EVALUATION_REFERENCE"
    if not allowed:
        raise ValueError(f"mode/execution/path combination is not allowed: "
                         f"{mode}/{execution}/{path}")
    cell["path"] = path
    expected_namespace = {
        "AUTO_DISCOVERY": "AUTO_DISCOVERY",
        "FIXED_PARTITION_DEBUG": "FIXED_PARTITION_DEBUG",
    }.get(path)
    if (cell.get("cache_namespace") and expected_namespace and
            cell["cache_namespace"] != expected_namespace):
        raise ValueError("mode/execution/path cache namespace mismatch")
    if expected_namespace and execution in {
            "CACHE_PRODUCER", "AUTOMATIC_STAGE2_TRAJECTORY",
            "CACHE_DIAGNOSTIC", "FINAL_TRAJECTORY"}:
        cell["cache_namespace"] = expected_namespace
    if mode in POLICY_THRESHOLD_KEYS:
        thresholds = cell.get("thresholds")
        if execution in {"CACHE_DIAGNOSTIC", "FINAL_TRAJECTORY"}:
            if not isinstance(thresholds, dict) or set(thresholds) != POLICY_THRESHOLD_KEYS[mode]:
                raise ValueError(f"{mode} requires exactly thresholds "
                                 f"{sorted(POLICY_THRESHOLD_KEYS[mode])}")
            if any(not isinstance(value, (int, float)) or
                   not math.isfinite(float(value)) for value in thresholds.values()):
                raise ValueError("policy thresholds must be finite numbers")
            if any(float(value) < 0.0 for value in thresholds.values()):
                raise ValueError("policy thresholds must be nonnegative")
            if "tau_eta" in thresholds and float(thresholds["tau_eta"]) > 1.0:
                raise ValueError("tau_eta must be in [0,1]")
    if execution in {"CACHE_DIAGNOSTIC", "FINAL_TRAJECTORY"}:
        if not cell.get("producer_id"):
            # Legacy v1 manifests bind to the sole producer in the run unit.
            cell["producer_id"] = "default"
        if not cell.get("operating_point_id"):
            cell["operating_point_id"] = identity(
                "t09point", {"mode": mode, "thresholds": cell.get("thresholds", {})})


def normalize_ldd_output(text: str) -> str:
    """Remove ASLR addresses while retaining stable SONAME/path identities."""
    normalized = []
    for raw in text.splitlines():
        words = [word for word in raw.replace("=>", " => ").split()
                 if not (word.startswith("(0x") and word.endswith(")"))]
        normalized.append(" ".join(words))
    return "\n".join(sorted(line for line in normalized if line))


def canonical_bytes(value: object) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False).encode()


def identity(prefix: str, value: object) -> str:
    return f"{prefix}-sha256:" + hashlib.sha256(canonical_bytes(value)).hexdigest()


def lossless_thresholds_sha256(mode: str, thresholds: dict) -> str:
    """Canonical binary64 identity for the actual three-slot gate input."""
    values = {
        "tau_eta": float(thresholds.get("tau_eta", 0.0)),
        "tau_s_m": float(thresholds.get("tau_s_m", 0.0)),
        "tau_gamma": float(thresholds.get("tau_gamma", 0.0)),
    }
    text = "uifgo-t09-thresholds-binary64-v1\n" + "".join(
        f"{name}={struct.pack('>d', values[name]).hex()}\n"
        for name in ("tau_eta", "tau_s_m", "tau_gamma"))
    return "sha256:" + hashlib.sha256(text.encode("ascii")).hexdigest()


def final_request_identity(value: dict) -> str:
    fields = [value[name] for name in (
        "stage2_cache_id", "canonical_mode", "policy_version",
        "thresholds_sha256", "threshold_provenance",
        "final_refit_score_config_sha256", "solver_sha256",
        "common_preparation_id")]
    encoded = bytearray(b"uifgo-t09-final-request-v1\n")
    for field in fields:
        raw = str(field).encode("utf-8")
        encoded.extend(str(len(raw)).encode("ascii"))
        encoded.extend(b":")
        encoded.extend(raw)
        encoded.extend(b"\n")
    return "t09finalrequest-sha256:" + hashlib.sha256(encoded).hexdigest()


def stage2_producer_config_identity(config: dict, base: Path) -> str:
    """Hash only Stage1/2/3 semantics; final gate points are reusable."""
    nlos = copy.deepcopy(config.get("nlos", {}))
    for key in FINAL_ONLY_NLOS_KEYS:
        nlos.pop(key, None)
    support = nlos.get("oracle_support")
    if isinstance(support, str) and support:
        path = Path(support)
        if not path.is_absolute():
            path = (base / path).resolve()
        nlos["oracle_support"] = {
            "canonical_path": str(path),
            "sha256": file_sha(path) if path.is_file() else "MISSING",
        }
    return identity("t09stage2config", {"nlos": nlos})


def common_config_identity(config: dict) -> str:
    return identity("t09commonconfig", common_config_view(config))


def stage2_cache_identity(value: dict) -> str:
    """Byte-for-byte equivalent of C++ ComputeStage2CacheId."""
    fields = [
        value["cache_namespace"], value["debug_label"],
        value["common_preparation_id"], value["source_identity"],
        value["producer_common_config_sha256"],
        value["producer_stage2_config_sha256"],
        value["stage1_config_sha256"], value["stage2_refit_config_sha256"],
        value["stage3_score_config_sha256"],
        value["support_partition_sha256"], value["observation_mapping_sha256"],
        value["stage2_graph_linearization_sha256"],
        value["stage2_values_sha256"], value["factor_metadata_sha256"],
        value["stage2_trace_sha256"], value["score_table_sha256"],
        value["producer_commit"], value["producer_binary_sha256"],
        value["producer_abi_sha256"], value["producer_toolchain_sha256"],
        value["stage2_status"], value["score_status"],
    ]
    for payload in value["payloads"]:
        fields.extend((payload["name"], payload["sha256"]))
    encoded = bytearray(b"uifgo-t09-stage2-cache-v2\n")
    for field in fields:
        raw = str(field).encode("utf-8")
        encoded.extend(str(len(raw)).encode("ascii"))
        encoded.extend(b":")
        encoded.extend(raw)
        encoded.extend(b"\n")
    return "t09stage2cache-sha256:" + hashlib.sha256(encoded).hexdigest()


def file_sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return "sha256:" + digest.hexdigest()


def producer_provenance(runner: Path) -> dict:
    repository = Path(__file__).resolve().parents[2]
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=repository, text=True,
        capture_output=True, check=False)
    if commit.returncode != 0 or not commit.stdout.strip():
        raise ValueError("cannot resolve producer git commit")
    abi = subprocess.run(["ldd", str(runner)], text=True,
                         capture_output=True, check=False)
    if abi.returncode == 0:
        abi_lines = []
        for line in normalize_ldd_output(abi.stdout).splitlines():
            words = line.replace("=>", " ").split()
            paths = [Path(word) for word in words if word.startswith("/")]
            identities = [file_sha(path) for path in paths if path.is_file()]
            abi_lines.append(line + " " + " ".join(identities))
        abi_text = "\n".join(abi_lines)
    else:
        abi_text = "NOT_DYNAMIC_ELF\n" + abi.stderr
    return {
        "producer_commit": commit.stdout.strip(),
        "producer_binary_sha256": file_sha(runner),
        "producer_abi_sha256": identity("t09abi", abi_text),
        "producer_toolchain_sha256": identity("t09toolchain", {
            "runner": file_sha(runner),
            "batch_scheduler": file_sha(Path(__file__).resolve()),
            "source_contents": {str(p.relative_to(repository)): file_sha(p)
                for directory in ("src", "include", "tools")
                for p in sorted((repository / directory).rglob("*"))
                if p.is_file() and p.suffix in {".cpp", ".h", ".py"}},
        }),
    }


def atomic_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    staging = path.with_suffix(path.suffix + ".staging")
    with staging.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, sort_keys=True, indent=2, allow_nan=False)
        stream.write("\n")
    os.replace(staging, path)


def resolve_config_paths(config: dict, base: Path) -> None:
    candidates = [("bag", "path"), ("dataset", "cache_manifest"),
                  ("nlos", "oracle_support"), ("sfuise", "data_dir"), ("miluv", "data_dir")]
    for section, key in candidates:
        value = config.get(section, {}).get(key)
        if isinstance(value, str) and value and not Path(value).is_absolute():
            config[section][key] = str((base / value).resolve())


def strip_truth(config: dict, fixed_debug: bool) -> dict:
    output = copy.deepcopy(config)
    topics = output.get("topics")
    if isinstance(topics, dict):
        for key in list(topics):
            if key.lower().startswith(("gt", "vicon", "truth", "label", "oracle")):
                del topics[key]
    for key in list(output):
        if key.lower() in {"truth", "ground_truth", "labels", "oracle"}:
            del output[key]
    if not fixed_debug and isinstance(output.get("nlos"), dict):
        output["nlos"].pop("oracle_support", None)
    return output


def common_config_view(config: dict) -> dict:
    value = copy.deepcopy(config)
    value.pop("nlos", None)
    return value


def load_manifest(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        manifest = yaml.safe_load(stream)
    if not isinstance(manifest, dict) or manifest.get("schema") not in {
            "uifgo_t09_batch_v1", "uifgo_t09_batch_v2"}:
        raise ValueError("manifest schema must be uifgo_t09_batch_v1/v2")
    if manifest.get("role") != "development":
        raise ValueError("T09 review-fix runner is development-only; validation/test is not authorized")
    units = manifest.get("run_units")
    if not isinstance(units, list) or not units:
        raise ValueError("manifest run_units must be a nonempty list")
    seen_units: set[str] = set()
    for unit in units:
        required = ("run_unit_id", "recording_id", "base_trajectory_id",
                    "seed", "prefix_identity", "config", "cells")
        if not isinstance(unit, dict) or any(key not in unit for key in required):
            raise ValueError("run unit is missing a required field")
        if unit["run_unit_id"] in seen_units:
            raise ValueError("duplicate run_unit_id")
        seen_units.add(unit["run_unit_id"])
        anchor_ids = unit.get("anchor_ids")
        if anchor_ids is not None:
            if (not isinstance(anchor_ids, list) or not anchor_ids or
                    any(not isinstance(value, int) for value in anchor_ids) or
                    len(set(anchor_ids)) != len(anchor_ids)):
                raise ValueError("run unit anchor_ids must be a nonempty unique integer list")
            unit["anchor_ids"] = sorted(anchor_ids)
        if not isinstance(unit["cells"], list) or not unit["cells"]:
            raise ValueError("run unit cells must be nonempty")
        seen_cells: set[tuple[str, str, str]] = set()
        for raw in unit["cells"]:
            cell = {"mode": raw} if isinstance(raw, str) else raw
            if not isinstance(cell, dict) or cell.get("mode") not in CANONICAL_MODES:
                raise ValueError("unknown canonical mode")
            execution = cell.get("execution_type")
            if execution not in EXECUTION_TYPES:
                raise ValueError("cell execution_type is invalid")
            if manifest.get("schema") == "uifgo_t09_batch_v2" and not cell.get("path"):
                raise ValueError("v2 cells require an explicit execution path")
            validate_mode_execution_path(cell)
            key = (cell["mode"], execution,
                   str(cell.get("operating_point_id", "")))
            if key in seen_cells:
                raise ValueError("duplicate mode/execution/operating-point cell in run unit")
            seen_cells.add(key)
            numeric_policy = any(name in cell for name in (
                "robust_scale", "rejection_threshold_sigma", "thresholds"))
            provenance = cell.get(
                "parameter_provenance", manifest.get("parameter_provenance"))
            if numeric_policy and not isinstance(provenance, str):
                raise ValueError("development policy parameters require provenance")
            if numeric_policy and not any(marker in provenance for marker in
                                          ("TEST_ONLY", "PENDING_VALIDATION")):
                raise ValueError("development policy provenance is not authorized")
            if "robust_scale" in cell and (not math.isfinite(float(cell["robust_scale"])) or
                                           float(cell["robust_scale"]) <= 0.0):
                raise ValueError("robust_scale must be finite and positive")
            if "rejection_threshold_sigma" in cell and (
                    not math.isfinite(float(cell["rejection_threshold_sigma"])) or
                    float(cell["rejection_threshold_sigma"]) < 0.0):
                raise ValueError("rejection threshold must be finite and nonnegative")
            if (cell["mode"] == "eta_only" and
                    execution == "FINAL_TRAJECTORY" and
                    cell.get("synthetic_final") is not True):
                raise ValueError(
                    "eta_only final trajectory must be explicitly predeclared synthetic_final")
    return manifest


def validate_final_success(run_dir: Path, expected_mode: str,
                           expected_cache_id: str | None = None,
                           expected_operating_point_id: str | None = None,
                           expected_thresholds: dict | None = None) -> tuple[bool, str]:
    required = ["run_status.json", "run_manifest.json",
                "final_inference_summary.json", "final_content_identity.json",
                "final_masks.csv", "final_factor_audit.csv", "trajectory.tum"]
    if any(not (run_dir / name).is_file() for name in required):
        return False, "FINAL_ARTIFACT_SET_INCOMPLETE"
    status = read_json(run_dir / "run_status.json") or {}
    manifest = read_json(run_dir / "run_manifest.json") or {}
    summary = read_json(run_dir / "final_inference_summary.json") or {}
    content = read_json(run_dir / "final_content_identity.json") or {}
    inference = status.get("inference_id")
    request = status.get("final_request_id")
    if (not status.get("valid_estimate_exported") or not summary.get("valid_estimate") or
            status.get("exit_code") != 0):
        return False, "FINAL_VALID_ESTIMATE_FALSE"
    if manifest.get("canonical_mode") != expected_mode:
        return False, "FINAL_CANONICAL_MODE_MISMATCH"
    if (expected_cache_id is not None and
            manifest.get("stage2_cache_id") != expected_cache_id):
        return False, "FINAL_PARENT_CACHE_MISMATCH"
    if (expected_operating_point_id is not None and
            manifest.get("operating_point_id") != expected_operating_point_id):
        return False, "FINAL_OPERATING_POINT_MISMATCH"
    if (not inference or inference == "UNAVAILABLE:FINAL_ENGINE_NOT_ENTERED" or
            not request or any(document.get("inference_id") != inference
                               for document in (manifest, summary, content))):
        return False, "FINAL_IDENTITY_MISMATCH"
    if manifest.get("export_verification_status") != "VERIFIED":
        return False, "FINAL_EXPORT_NOT_VERIFIED"
    if summary.get("factor_audit_status") != "OK":
        return False, "FINAL_FACTOR_AUDIT_FAILED"
    if manifest.get("final_request_id") != request:
        return False, "FINAL_REQUEST_ID_MISMATCH"
    if expected_thresholds is not None:
        expected_threshold_hash = lossless_thresholds_sha256(
            expected_mode, expected_thresholds)
        if manifest.get("thresholds_sha256") != expected_threshold_hash:
            return False, "FINAL_THRESHOLD_IDENTITY_MISMATCH"
        if lossless_thresholds_sha256(expected_mode, summary) != expected_threshold_hash:
            return False, "FINAL_SUMMARY_THRESHOLD_MISMATCH"
    request_fields = {
        name: manifest.get(name) for name in (
            "stage2_cache_id", "canonical_mode", "policy_version",
            "thresholds_sha256", "threshold_provenance",
            "final_refit_score_config_sha256", "solver_sha256",
            "common_preparation_id")}
    if any(not value for value in request_fields.values()):
        return False, "FINAL_REQUEST_PROVENANCE_INCOMPLETE"
    if final_request_identity(request_fields) != request:
        return False, "FINAL_REQUEST_CONTENT_MISMATCH"
    return True, "OK"


def terminal(status: str) -> bool:
    return status not in {"PREREGISTERED", "RUNNING"}


def read_json(path: Path) -> dict | None:
    try:
        with path.open(encoding="utf-8") as stream:
            value = json.load(stream)
        return value if isinstance(value, dict) else None
    except (OSError, ValueError):
        return None


def time_rss(path: Path) -> int | None:
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            if "Maximum resident set size" in line:
                return int(line.rsplit(":", 1)[1].strip())
    except (OSError, ValueError):
        pass
    return None


def runner_call(runner: Path, config: Path, output_root: Path, run_id: str,
                method: str | None, execution_type: str,
                cache_manifest: Path | None,
                operating_point_id: str | None,
                anchor_ids: list[int] | None,
                env_extra: dict[str, str]) -> tuple[int, float, str, str, int | None]:
    command = ["/usr/bin/time", "-v", "-o", str(output_root / f".{run_id}.time"),
               str(runner), "--config", str(config), "--output-root",
               str(output_root), "--run-id", run_id,
               "--execution-type", execution_type]
    if method:
        command += ["--method", method]
    if cache_manifest:
        command += ["--stage2-cache-manifest", str(cache_manifest)]
    if operating_point_id:
        command += ["--operating-point-id", operating_point_id]
    if anchor_ids:
        command += ["--anchor-ids", ",".join(str(value) for value in anchor_ids)]
    env = os.environ.copy()
    env.update(env_extra)
    start = time.monotonic()
    completed = subprocess.run(command, text=True, capture_output=True, env=env,
                               check=False)
    wall = time.monotonic() - start
    run_directory = output_root / run_id
    if run_directory.is_dir():
        atomic_json(run_directory / "command.json", {"command":command, "exit_code":completed.returncode})
    resource = output_root / f".{run_id}.time"
    rss = time_rss(resource)
    resource.unlink(missing_ok=True)
    return completed.returncode, wall, completed.stdout, completed.stderr, rss


def publish_cache(run_dir: Path, cache_root: Path, namespace: str,
                  common_id: str, common_config_sha256: str,
                  stage2_config_sha256: str, fixed_debug: bool,
                  producer: dict) -> dict:
    status = read_json(run_dir / "run_status.json") or {}
    capability = read_json(run_dir / "capability_status.json") or {}
    if not (run_dir / "trajectory.tum").is_file():
        raise RuntimeError("Stage-2 cache producer has no trajectory")
    payload_names = [name for name in (
        "input_manifest.json", "observations.csv", "partition.json",
        "support_snapshot.csv", "segments.csv", "factor_metadata.csv",
        "refit_iterations.csv", "scores_decision.csv", "segment_fit_scores.csv",
        "groups.csv", "trajectory.tum", "imu_bias.csv", "stage2_values.csv",
        "stage2_content_identity.json", "stage2_producer_context.json")
        if (run_dir / name).is_file()]
    required = STAGE2_REPLAY_REQUIRED_PAYLOADS | {"trajectory.tum", "imu_bias.csv"}
    if not required.issubset(payload_names):
        raise RuntimeError("Stage-2 cache producer payload is incomplete")
    groups = list(csv.DictReader((run_dir / "groups.csv").open(
        newline="", encoding="utf-8")))
    scores = list(csv.DictReader((run_dir / "scores_decision.csv").open(
        newline="", encoding="utf-8")))
    if {row.get("group_id") for row in groups} != {
            row.get("group_id") for row in scores}:
        raise RuntimeError("Stage-2 score table does not cover every frozen group")
    if not fixed_debug and capability.get("discovery") not in {
            "CONVERGED_STAGE1", "NO_CANDIDATES"}:
        raise RuntimeError("automatic cache producer did not pass Stage-1 admission")
    payloads = [{"name": name, "sha256": file_sha(run_dir / name)}
                for name in sorted(payload_names)]
    score_status = ("COMPLETE" if status.get("exit_code") == 0 else
                    "COMPLETE_WITH_SCORE_UNAVAILABLE")
    producer_context = read_json(run_dir / "stage2_producer_context.json") or {}
    if producer_context.get("schema") != \
            "uifgo_t09_stage2_producer_context_v1":
        raise RuntimeError("Stage-2 producer context schema mismatch")
    if any(not str(producer_context.get(name, "")).startswith("sha256:")
           for name in ("stage1_config_sha256", "stage2_refit_config_sha256",
                        "stage3_score_config_sha256")):
        raise RuntimeError("Stage-2 producer configuration identity is incomplete")
    core = {
        "schema": "uifgo_t09_stage2_cache_v2",
        "cache_namespace": namespace,
        "debug_label": ("RQ3_FIXED_PARTITION_DIAGNOSTIC_DEBUG_ONLY"
                        if fixed_debug else "RQ3_AUTO_DISCOVERY_END_TO_END"),
        "common_preparation_id": common_id,
        "source_identity": (read_json(run_dir / "input_manifest.json") or {}).get(
            "source_hash_sha256", "UNAVAILABLE"),
        "producer_common_config_sha256": common_config_sha256,
        "producer_stage2_config_sha256": stage2_config_sha256,
        "stage1_config_sha256": producer_context.get(
            "stage1_config_sha256", "UNAVAILABLE"),
        "stage2_refit_config_sha256": producer_context.get(
            "stage2_refit_config_sha256", "UNAVAILABLE"),
        "stage3_score_config_sha256": producer_context.get(
            "stage3_score_config_sha256", "UNAVAILABLE"),
        "support_partition_sha256": file_sha(run_dir / "segments.csv"),
        "observation_mapping_sha256": file_sha(run_dir / "observations.csv"),
        "stage2_graph_linearization_sha256": (read_json(
            run_dir / "stage2_content_identity.json") or {}).get(
                "graph_linearization_sha256", "UNAVAILABLE"),
        "stage2_values_sha256": (read_json(
            run_dir / "stage2_content_identity.json") or {}).get(
                "values_sha256", "UNAVAILABLE"),
        "factor_metadata_sha256": file_sha(run_dir / "factor_metadata.csv"),
        "stage2_trace_sha256": file_sha(run_dir / "refit_iterations.csv"),
        "score_table_sha256": file_sha(run_dir / "scores_decision.csv"),
        "stage2_status": "CONVERGED",
        "score_status": score_status,
        "producer_status": status.get("status", "UNKNOWN"),
        "producer_capability": capability.get("recoverability_score", "UNKNOWN"),
        **producer,
        "payloads": payloads,
    }
    cache_id = stage2_cache_identity(core)
    final = cache_root / cache_id.split(":", 1)[1]
    if final.exists():
        existing = read_json(final / "stage2_cache_manifest.json")
        if existing and existing.get("cache_id") == cache_id:
            return existing
        raise RuntimeError("cache identity collision or attempted overwrite")
    staging = cache_root / (".staging-" + uuid.uuid4().hex)
    staging.mkdir(parents=True)
    for payload in payloads:
        shutil.copy2(run_dir / payload["name"], staging / payload["name"])
    result = dict(core, cache_id=cache_id)
    atomic_json(staging / "stage2_cache_manifest.json", result)
    os.replace(staging, final)
    return result


def validate_cache_compatibility(cache: dict, cell: dict) -> None:
    if cache.get("schema") != "uifgo_t09_stage2_cache_v2":
        raise RuntimeError("CACHE_SCHEMA_INCOMPATIBLE_REQUIRES_REPRODUCTION")
    if cache.get("producer_common_config_sha256") != cell.get(
            "requested_common_config_sha256"):
        raise RuntimeError("CACHE_COMMON_CONFIG_INCOMPATIBLE")
    if cache.get("producer_stage2_config_sha256") != cell.get(
            "requested_stage2_config_sha256"):
        raise RuntimeError("CACHE_STAGE1_STAGE2_STAGE3_CONFIG_INCOMPATIBLE")


def parse_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes"}


def diagnostic(cache_manifest: dict, cache_dir: Path, mode: str,
               thresholds: dict, output: Path) -> dict:
    score_rows = list(csv.DictReader((cache_dir / "scores_decision.csv").open(
        newline="", encoding="utf-8")))
    fit_rows = list(csv.DictReader((cache_dir / "segment_fit_scores.csv").open(
        newline="", encoding="utf-8"))) if (cache_dir / "segment_fit_scores.csv").is_file() else []
    segments = list(csv.DictReader((cache_dir / "segments.csv").open(
        newline="", encoding="utf-8")))
    gamma_by_group: dict[str, list[float]] = {}
    for row in fit_rows:
        try:
            gamma_by_group.setdefault(row["group_id"], []).append(float(row["gamma"]))
        except (KeyError, ValueError):
            pass
    segment_count_by_ordinal = {row.get("segment_ordinal", ""): int(row["obs_count"])
                                for row in segments}
    decisions = []
    for row in score_rows:
        eligible = parse_bool(row.get("eligible", "false"))
        availability = row.get("score_availability", "UNAVAILABLE")
        eta = float(row["eta"]) if row.get("eta") else None
        s_value = float(row["s_m"]) if row.get("s_m") else None
        s_inf = parse_bool(row.get("s_is_infinite", "false"))
        gammas = gamma_by_group.get(row.get("group_id", ""), [])
        gamma = max(gammas) if gammas else None
        eta_ok = eta is not None and eta >= float(thresholds.get("tau_eta", 0.0))
        s_ok = s_value is not None and not s_inf and s_value <= float(thresholds.get("tau_s_m", 0.0))
        gamma_ok = gamma is not None and gamma <= float(thresholds.get("tau_gamma", 0.0))
        required_available = False
        passed = False
        if mode == "fit_only":
            required_available, passed = gamma is not None, gamma_ok
        elif mode == "s_fit":
            required_available, passed = (s_value is not None or s_inf) and gamma is not None, s_ok and gamma_ok
        elif mode == "full_gate":
            required_available = eta is not None and (s_value is not None or s_inf) and gamma is not None
            passed = eta_ok and s_ok and gamma_ok
        elif mode == "eta_only":
            required_available, passed = eta is not None, eta_ok
        else:
            nominal = (float(row["lambda_min_N_m2_inv"])
                       if row.get("lambda_min_N_m2_inv") else None)
            required_available = nominal is not None
            passed = required_available and nominal >= float(
                thresholds.get("tau_N_m2_inv", 0.0))
        if not eligible:
            decision, reason = "SUPPRESS", "SUPPRESS_INELIGIBLE_" + row.get("status", "UNKNOWN")
        elif not required_available:
            decision, reason = "SUPPRESS", "SUPPRESS_REQUIRED_SCORE_UNAVAILABLE"
        elif passed:
            decision, reason = "USE", "USE_ALL_REQUIRED_PREDICATES_PASS"
        else:
            decision, reason = "SUPPRESS", "SUPPRESS_POLICY_PREDICATE_FAILED"
        decisions.append({
            "group_id": row.get("group_id"), "decision": decision,
            "reason": reason, "eligible": eligible, "eta_pass": eta_ok,
            "s_pass": s_ok, "gamma_pass": gamma_ok,
            "score_availability": availability,
            "linearization_id": row.get("linearization_id"),
            "segment_ordinals": row.get("segment_ordinals", ""),
        })
    accepted_groups = {row["group_id"] for row in decisions if row["decision"] == "USE"}
    group_rows = list(csv.DictReader((cache_dir / "groups.csv").open(
        newline="", encoding="utf-8"))) if (cache_dir / "groups.csv").is_file() else []
    candidate = sum(segment_count_by_ordinal.values())
    eligible = 0
    accepted = 0
    for group in group_rows:
        count = sum(segment_count_by_ordinal.get(item, 0)
                    for item in group.get("segment_ordinals", "").split(";") if item)
        if parse_bool(group.get("eligible", "false")):
            eligible += count
        if group.get("group_id") in accepted_groups:
            accepted += count
    def ratio(num: int, den: int) -> dict:
        return ({"value": num / den, "status": "AVAILABLE", "numerator": num,
                 "denominator": den, "unit": "observation", "reason": ""}
                if den else {"value": None, "status": "UNDEFINED_ZERO_DENOMINATOR",
                             "numerator": num, "denominator": den,
                             "unit": "observation", "reason": "ZERO_DENOMINATOR"})
    result = {
        "schema": "uifgo_t09_cache_diagnostic_v1", "mode": mode,
        "cache_id": cache_manifest["cache_id"],
        "cache_namespace": cache_manifest["cache_namespace"],
        "execution_type": "CACHE_DIAGNOSTIC", "decisions": decisions,
        "candidate_use_coverage": ratio(accepted, candidate),
        "eligible_use_coverage": ratio(accepted, eligible),
        "overall_retained_fraction": {
            "value": None, "status": "NOT_APPLICABLE_NO_FINAL_GRAPH",
            "numerator": None, "denominator": None, "unit": "observation",
            "reason": "DIAGNOSTIC_ONLY_CELL"},
    }
    atomic_json(output / "diagnostic.json", result)
    with (output / "decisions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(decisions[0]) if decisions else
                                ["group_id", "decision", "reason"])
        writer.writeheader()
        writer.writerows(decisions)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--runner", required=True)
    parser.add_argument("--output-root", required=True)
    args = parser.parse_args()
    manifest_path = Path(args.manifest).resolve()
    runner = Path(args.runner).resolve()
    output_root = Path(args.output_root).resolve()
    try:
        manifest = load_manifest(manifest_path)
        if not runner.is_file():
            raise ValueError("runner does not exist")
        producer = producer_provenance(runner)
        if output_root.exists():
            raise ValueError("output root already exists")
        output_root.mkdir(parents=True)
        (output_root / "runs").mkdir()
        (output_root / "caches").mkdir()
        cells = []
        units_by_id = {}
        for unit in manifest["run_units"]:
            units_by_id[unit["run_unit_id"]] = unit
            source_config = (manifest_path.parent / unit["config"]).resolve()
            if not source_config.is_file():
                raise ValueError(f"config does not exist: {source_config}")
            config_doc = yaml.safe_load(source_config.read_text(encoding="utf-8"))
            resolve_config_paths(config_doc, source_config.parent)
            estimator_config = strip_truth(config_doc, False)
            common_id = identity("t09common", {
                "run_unit": {key: unit[key] for key in (
                    "recording_id", "base_trajectory_id", "seed", "prefix_identity")},
                "anchor_ids": unit.get("anchor_ids", []),
                "config": common_config_view(estimator_config),
            })
            for raw in unit["cells"]:
                spec = {"mode": raw} if isinstance(raw, str) else dict(raw)
                fixed_debug = spec.get("path") == "FIXED_PARTITION_DEBUG"
                requested_config = strip_truth(config_doc, fixed_debug)
                requested_common_config = common_config_identity(requested_config)
                requested_stage2_config = stage2_producer_config_identity(
                    requested_config, source_config.parent)
                cell_id = identity("t09cell", {
                    "run_unit_id": unit["run_unit_id"], "mode": spec["mode"],
                    "execution_type": spec["execution_type"],
                    "path": spec["path"],
                    "operating_point_id": spec.get("operating_point_id", "")})
                cells.append({
                    "cell_id": cell_id, "run_unit_id": unit["run_unit_id"],
                    "canonical_mode": spec["mode"],
                    "execution_type": spec["execution_type"],
                    "path": spec["path"],
                    "producer_id": spec.get("producer_id"),
                    "anchor_ids": unit.get("anchor_ids", []),
                    "operating_point_id": spec.get("operating_point_id"),
                    "common_preparation_request_id": common_id,
                    "requested_common_config_sha256": requested_common_config,
                    "requested_stage2_config_sha256": requested_stage2_config,
                    "common_preparation_id": "PENDING_ACTUAL_PREPARATION",
                    "cache_namespace": spec.get("cache_namespace"),
                    "parameter_provenance": spec.get(
                        "parameter_provenance", manifest.get("parameter_provenance")),
                    "status": "PREREGISTERED", "attempts": [],
                    "config": str(source_config), "spec": spec,
                })
        batch = {
            "schema": "uifgo_t09_batch_manifest_v2",
            "source_manifest_sha256": file_sha(manifest_path),
            "role": manifest["role"], "status": "RUNNING", "cells": cells,
            **producer,
            "created_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        }
        atomic_json(output_root / "batch_manifest.json", batch)
        producer_by_key: dict[tuple[str, str], tuple[dict, Path]] = {}
        for external in manifest.get("external_caches", []):
            if not isinstance(external, dict) or not external.get("producer_id") or not external.get("manifest"):
                raise ValueError("external cache requires producer_id and manifest")
            external_path = Path(external["manifest"])
            if not external_path.is_absolute():
                external_path = (manifest_path.parent / external_path).resolve()
            cache = read_json(external_path)
            if cache is None or cache.get("schema") != "uifgo_t09_stage2_cache_v2":
                raise ValueError("external Stage-2 cache manifest is invalid")
            if cache.get("cache_id") != stage2_cache_identity(cache):
                raise ValueError("external Stage-2 cache identity mismatch")
            for field in ("producer_commit", "producer_binary_sha256",
                          "producer_abi_sha256", "producer_toolchain_sha256"):
                if cache.get(field) != producer.get(field):
                    raise ValueError(f"external Stage-2 cache invalidated by {field}")
            for payload in cache.get("payloads", []):
                payload_path = external_path.parent / payload.get("name", "")
                if not payload_path.is_file() or file_sha(payload_path) != payload.get("sha256"):
                    raise ValueError("external Stage-2 cache payload verification failed")
            key = (str(external.get("run_unit_id", "*")), external["producer_id"])
            if key in producer_by_key:
                raise ValueError("duplicate external cache producer_id")
            producer_by_key[key] = (cache, external_path.parent)
        common_by_unit: dict[str, str] = {}
        ordered = sorted(cells, key=lambda cell: (cell["run_unit_id"],
                         0 if cell["canonical_mode"] in BASELINES else
                         1 if cell["canonical_mode"] in {"structured_bias_only", "structured_debias"} else 2))
        for cell in ordered:
            unit = units_by_id[cell["run_unit_id"]]
            mode = cell["canonical_mode"]
            spec = cell["spec"]
            if mode == "oracle_reference":
                cell["status"] = "COMPLETE_EVALUATION_REFERENCE_NOT_ESTIMATOR"
                continue
            if mode in DIAGNOSTICS and cell["execution_type"] == "CACHE_DIAGNOSTIC":
                producer_id = str(cell.get("producer_id") or "default")
                parent = (producer_by_key.get((cell["run_unit_id"], producer_id)) or
                          producer_by_key.get(("*", producer_id)))
                if parent is None:
                    cell["status"] = "PARENT_CACHE_UNAVAILABLE"
                    cell["reason"] = "NO_ADMITTED_STAGE2_CACHE"
                    continue
                cache, cache_dir = parent
                validate_cache_compatibility(cache, cell)
                requested_namespace = spec.get("cache_namespace")
                if (requested_namespace and requested_namespace !=
                        cache.get("cache_namespace")):
                    raise RuntimeError("diagnostic cache namespace mismatch")
                run_id = "t09diag-" + uuid.uuid4().hex
                run_dir = output_root / "runs" / run_id
                run_dir.mkdir()
                diagnostic(cache, cache_dir, mode, spec.get("thresholds", {}), run_dir)
                cell.update(status="COMPLETE", run_id=run_id,
                            cache_id=cache["cache_id"], run_directory=str(run_dir))
                cell["parent_cache_manifest"] = str(
                    cache_dir / "stage2_cache_manifest.json")
                cell["cache_namespace"] = cache["cache_namespace"]
                cell["common_preparation_id"] = cache["common_preparation_id"]
                continue

            source = Path(cell["config"])
            raw_config = yaml.safe_load(source.read_text(encoding="utf-8"))
            fixed_debug = spec.get("cache_namespace") == "FIXED_PARTITION_DEBUG"
            resolve_config_paths(raw_config, source.parent)
            effective = strip_truth(raw_config, fixed_debug)
            if mode in BASELINES:
                effective.setdefault("nlos", {})["mode"] = "disabled"
                effective["nlos"]["score_recoverability"] = False
                effective["nlos"]["final_inference_enabled"] = False
            elif cell["execution_type"] in {"CACHE_PRODUCER",
                                             "AUTOMATIC_STAGE2_TRAJECTORY"}:
                effective.setdefault("nlos", {})["final_inference_enabled"] = False
            elif cell["execution_type"] == "FINAL_TRAJECTORY":
                # The dedicated cache-replay entry point controls Stage 4;
                # leaving this false prevents the legacy inline Stage1/2 path.
                effective.setdefault("nlos", {})["final_inference_enabled"] = False
                thresholds = spec.get("thresholds", {})
                # Config validation retains the T08 three-threshold shape; a
                # policy consumes only the fields declared by its truth table.
                effective["nlos"]["tau_eta"] = float(thresholds.get("tau_eta", 0.0))
                effective["nlos"]["tau_s_m"] = float(thresholds.get("tau_s_m", 0.0))
                effective["nlos"]["tau_gamma"] = float(thresholds.get("tau_gamma", 0.0))
                effective["nlos"]["gate_parameter_provenance"] = (
                    "T08_GATE_DEVELOPMENT_ONLY_PENDING_VALIDATION")
            run_id = "t09-" + uuid.uuid4().hex
            cfg_dir = output_root / "effective_configs"
            cfg_dir.mkdir(exist_ok=True)
            effective_path = cfg_dir / f"{run_id}.yaml"
            effective_path.write_text(yaml.safe_dump(effective, sort_keys=True), encoding="utf-8")
            env_extra = {}
            if "robust_scale" in spec:
                env_extra["UIFGO_T09_ROBUST_SCALE"] = str(spec["robust_scale"])
            if "rejection_threshold_sigma" in spec:
                env_extra["UIFGO_T09_REJECTION_THRESHOLD_SIGMA"] = str(spec["rejection_threshold_sigma"])
            if mode == "eta_only" and cell["execution_type"] == "FINAL_TRAJECTORY":
                env_extra["UIFGO_T09_ETA_ONLY_SYNTHETIC_FINAL"] = "1"
            method_arg = (mode if mode in BASELINES or
                          mode == "structured_bias_only" or
                          cell["execution_type"] == "FINAL_TRAJECTORY" else None)
            parent_manifest = None
            parent_cache = None
            if cell["execution_type"] == "FINAL_TRAJECTORY":
                producer_id = str(cell.get("producer_id") or "default")
                parent = (producer_by_key.get((cell["run_unit_id"], producer_id)) or
                          producer_by_key.get(("*", producer_id)))
                if parent is None:
                    cell["status"] = "PARENT_CACHE_UNAVAILABLE"
                    cell["reason"] = "NO_ADMITTED_STAGE2_CACHE"
                    atomic_json(output_root / "batch_manifest.json", batch)
                    continue
                parent_cache, parent_dir = parent
                validate_cache_compatibility(parent_cache, cell)
                if parent_cache.get("cache_namespace") != cell.get("cache_namespace"):
                    raise RuntimeError("final cache namespace mismatch")
                parent_manifest = parent_dir / "stage2_cache_manifest.json"
                cell["cache_id"] = parent_cache["cache_id"]
                cell["parent_cache_manifest"] = str(parent_manifest)
            cell["status"] = "RUNNING"
            atomic_json(output_root / "batch_manifest.json", batch)
            code, wall, stdout, stderr, rss = runner_call(
                runner, effective_path, output_root / "runs", run_id,
                method_arg, cell["execution_type"], parent_manifest,
                cell.get("operating_point_id"), cell.get("anchor_ids"), env_extra)
            run_dir = output_root / "runs" / run_id
            if not run_dir.exists():
                raise RuntimeError("runner did not create its declared run directory")
            (run_dir / "stdout.log").write_text(stdout, encoding="utf-8")
            (run_dir / "stderr.log").write_text(stderr, encoding="utf-8")
            shutil.copy2(effective_path, run_dir / "batch_config_effective.yaml")
            status_doc = read_json(run_dir / "run_status.json")
            if status_doc is None:
                raise RuntimeError("runner produced missing or invalid run_status.json")
            common_doc = read_json(run_dir / "common_preparation.json")
            attempt = {"run_id": run_id, "exit_code": code,
                       "measured_wall_seconds": wall,
                       "measured_peak_rss_kib": rss,
                       "run_status": status_doc.get("status")}
            cell["attempts"].append(attempt)
            cell.update(run_id=run_id, run_directory=str(run_dir))
            if common_doc is None or not common_doc.get("common_preparation_id"):
                if code == 0:
                    raise RuntimeError("successful runner produced no common preparation identity")
                cell["status"] = "COMPLETE_WITH_RUN_FAILURE"
                cell["reason"] = "COMMON_PREPARATION_FAILED:" + status_doc.get("reason", "UNKNOWN")
                atomic_json(output_root / "batch_manifest.json", batch)
                continue
            actual_common = common_doc["common_preparation_id"]
            prior_common = common_by_unit.setdefault(cell["run_unit_id"],
                                                     actual_common)
            if prior_common != actual_common:
                cell["status"] = "COMPARABILITY_INVALID"
                cell["reason"] = "COMMON_PREPARATION_ID_MISMATCH"
                cell["common_preparation_id"] = actual_common
                atomic_json(output_root / "batch_manifest.json", batch)
                continue
            cell["common_preparation_id"] = actual_common
            if cell["execution_type"] == "FINAL_TRAJECTORY" and code == 0:
                final_ok, final_reason = validate_final_success(
                    run_dir, mode, cell.get("cache_id"),
                    cell.get("operating_point_id"), spec.get("thresholds"))
                if not final_ok:
                    raise RuntimeError(final_reason)
            if code == 0:
                cell["status"] = "COMPLETE"
            else:
                cell["status"] = "COMPLETE_WITH_RUN_FAILURE"
                cell["reason"] = status_doc.get("reason", "RUNNER_NONZERO_EXIT")
            if (mode == "structured_debias" and
                    cell["execution_type"] in {"CACHE_PRODUCER",
                                               "AUTOMATIC_STAGE2_TRAJECTORY"}):
                if (run_dir / "trajectory.tum").is_file() and (run_dir / "scores_decision.csv").is_file():
                    namespace = spec.get("cache_namespace", "AUTO_DISCOVERY")
                    cache = publish_cache(run_dir, output_root / "caches", namespace,
                                          actual_common,
                                          cell["requested_common_config_sha256"],
                                          cell["requested_stage2_config_sha256"],
                                          fixed_debug,
                                          producer)
                    cache_dir = output_root / "caches" / cache["cache_id"].split(":", 1)[1]
                    producer_id = str(cell.get("producer_id") or "default")
                    producer_key = (cell["run_unit_id"], producer_id)
                    if producer_key in producer_by_key:
                        raise RuntimeError("duplicate Stage-2 producer_id")
                    producer_by_key[producer_key] = (cache, cache_dir)
                    cell["cache_id"] = cache["cache_id"]
                    if cache["score_status"] == "COMPLETE_WITH_SCORE_UNAVAILABLE":
                        cell["cache_status"] = "COMPLETE_WITH_SCORE_UNAVAILABLE"
                        cell["status"] = "COMPLETE_WITH_SCORE_UNAVAILABLE"
                        cell.pop("reason", None)
                else:
                    cell["upstream_status"] = "PARENT_STAGE1_FAILED/" + str(
                        status_doc.get("discovery_status", status_doc.get("solver_status", "UNKNOWN")))
            atomic_json(output_root / "batch_manifest.json", batch)
        if not all(terminal(cell["status"]) for cell in cells):
            raise RuntimeError("not all preregistered cells reached a terminal state")
        failure_states = {"COMPLETE_WITH_RUN_FAILURE", "PARENT_CACHE_UNAVAILABLE"}
        if any(cell["status"] == "COMPARABILITY_INVALID" for cell in cells):
            batch["status"] = "INVALID_COMPARABILITY"
            batch["completed_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
            for cell in cells:
                cell.pop("spec", None)
            atomic_json(output_root / "batch_manifest.json", batch)
            raise RuntimeError("COMPARABILITY_INVALID batch cannot cross aggregation boundary")
        any_run_failure = any(cell["status"] in failure_states for cell in cells)
        batch["status"] = "COMPLETE_WITH_RUN_FAILURES" if any_run_failure else "COMPLETE"
        batch["completed_utc"] = dt.datetime.now(dt.timezone.utc).isoformat()
        batch["summary"] = {
            "preregistered_cells": len(cells),
            "terminal_cells": sum(terminal(cell["status"]) for cell in cells),
            "run_failure_cells": sum(cell["status"] in failure_states for cell in cells),
            "partial_score_cells": sum(cell["status"] ==
                                       "COMPLETE_WITH_SCORE_UNAVAILABLE"
                                       for cell in cells),
        }
        inferences: dict[str, set[str]] = {}
        for cell in cells:
            if not cell.get("run_directory"):
                continue
            status_doc = read_json(Path(cell["run_directory"]) / "run_status.json") or {}
            request_id = status_doc.get("final_request_id")
            inference_id = status_doc.get("inference_id")
            if request_id and inference_id:
                inferences.setdefault(request_id, set()).add(inference_id)
        batch["rerun_disagreements"] = [
            {"final_request_id": request, "inference_ids": sorted(ids)}
            for request, ids in sorted(inferences.items()) if len(ids) > 1]
        for cell in cells:
            cell.pop("spec", None)
        atomic_json(output_root / "batch_manifest.json", batch)
        return 0
    except Exception as error:  # infrastructure/schema failures are nonzero
        print(f"run_experiments ERROR: {error}", file=sys.stderr)
        if output_root.exists():
            failure = {"schema": "uifgo_t09_batch_failure_v1", "status": "INVALID",
                       "reason": str(error)}
            atomic_json(output_root / "batch_failure.json", failure)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
