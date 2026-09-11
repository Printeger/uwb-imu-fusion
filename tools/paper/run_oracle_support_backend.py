#!/usr/bin/env python3
"""Run the locked Walk1 oracle-support backend check without detector input."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

import yaml

import nlos_experiment_common as common


LOCKED_ROOT = Path("/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01")
BASELINE_HEAD = "15b32f5afa01c5a0f8bc432bbac78c53bfa36420"
METHODS = ("suppress_all", "structured_debias", "lcb_fixed_full", "lcb_partial")


def sha256(path: Path) -> str:
    return "sha256:" + hashlib.sha256(path.read_bytes()).hexdigest()


def scenario(locked: dict, scenario_id: str) -> dict:
    return next(item for item in locked["scenarios"]
                if item["scenario_id"] == scenario_id)


def make_cells() -> list[dict]:
    cells = [
        {"mode": "all_range", "execution_type": "BASELINE_TRAJECTORY",
         "path": "DIRECT_COMMON_PREPARATION"},
        {"mode": "robust_cauchy", "execution_type": "BASELINE_TRAJECTORY",
         "path": "DIRECT_COMMON_PREPARATION", "robust_scale": 2.3849},
        {"mode": "structured_debias", "execution_type": "CACHE_PRODUCER",
         "path": "FIXED_PARTITION_DEBUG", "producer_id": "oracle_shared",
         "cache_namespace": "FIXED_PARTITION_DEBUG"},
    ]
    cells.extend({"mode": method, "execution_type": "FINAL_TRAJECTORY",
                  "path": "FIXED_PARTITION_DEBUG", "producer_id": "oracle_shared",
                  "cache_namespace": "FIXED_PARTITION_DEBUG", "thresholds": {}}
                 for method in METHODS)
    return cells


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--regime", choices=("normal", "low"), default="normal")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)

    locked_path = LOCKED_ROOT / "locked_manifest.json"
    locked = json.loads(locked_path.read_text(encoding="utf-8"))
    normal = scenario(locked, "sfuise_walk1_normal_injected")
    truth_path = Path(normal["truth"]) / "nlos_injection_truth.json"
    truth = json.loads(truth_path.read_text(encoding="utf-8"))
    specification = truth["specification"]
    planned_ids = [str(value) for value in truth["planned_affected_obs_ids"]]
    if not planned_ids or len(planned_ids) != len(set(planned_ids)):
        raise ValueError("locked planned oracle support is empty or duplicated")

    if args.regime == "normal":
        source_config = Path(normal["config"])
        anchor_ids = list(specification["anchor_subset"])
        input_identity = truth["cache_id"]
    else:
        low = scenario(locked, "sfuise_walk1_low_injected")
        anchor_ids = list(low["subset"])
        if specification["anchor_id"] not in anchor_ids:
            raise ValueError("locked low-redundancy subset excludes target anchor")
        # The original matrix intentionally did not materialize this injected
        # cell after its clean robust baseline failed. Derive only the anchor
        # subset from the locked low cell and retain the immutable injected
        # cache, recipe, seed, link, interval and observation identities.
        source_config = Path(normal["config"])
        input_identity = truth["cache_id"]

    support_doc = {
        "schema": "t09_fixed_partition_v1",
        "RQ3_FIXED_PARTITION_DIAGNOSTIC_DEBUG_ONLY": True,
        "recipe_hash": truth["specification_sha256"],
        "segments": [{
            "segment_id": "oracle-walk1-persistent-nlos",
            "link": f'{specification["tag_id"]}:{specification["anchor_id"]}',
            "obs_ids": planned_ids,
        }],
    }
    support_path = output / "oracle_support.yaml"
    support_path.write_text(yaml.safe_dump(support_doc, sort_keys=False), encoding="utf-8")
    if any(key in support_path.read_text(encoding="utf-8")
           for key in ("amplitude", "bias_m", "c_true", "ground_truth")):
        raise AssertionError("oracle estimator manifest contains forbidden truth")

    support_json = {
        "schema": "uifgo_oracle_backend_support_v1",
        "segment_id": "oracle-walk1-persistent-nlos",
        "tag_id": specification["tag_id"],
        "anchor_id": specification["anchor_id"],
        "start_time": specification["start_s"],
        "end_time": specification["end_s"],
        "obs_ids": planned_ids,
        "obs_count": len(planned_ids),
        "support_source": "oracle_injection_interval",
        "estimator_fields": ["segment_id", "link", "obs_ids"],
        "bias_truth_available_to_estimator": False,
        "locked_manifest_sha256": sha256(locked_path),
        "truth_source_sha256": sha256(truth_path),
        "source_truth_read_phase": "PRE_ESTIMATOR_SUPPORT_CONSTRUCTION_ONLY",
    }
    (output / "oracle_support.json").write_text(
        json.dumps(support_json, indent=2) + "\n", encoding="utf-8")

    config = yaml.safe_load(source_config.read_text(encoding="utf-8"))
    nlos = config["nlos"]
    nlos["mode"] = "fixed_partition_debug"
    nlos["oracle_support"] = str(support_path)
    nlos["score_recoverability"] = True
    nlos["final_inference_enabled"] = False
    nlos["short_min_count_debug"] = nlos["discovery_short_min_count"]
    nlos["short_min_duration_debug"] = nlos["discovery_short_min_duration_s"]
    nlos.pop("fde_grouped_test", None)
    config_path = output / f"walk1_{args.regime}_oracle.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")

    unit = {
        "run_unit_id": f"sfuise_walk1_{args.regime}_oracle",
        "recording_id": "sfuise_walk1",
        "base_trajectory_id": "sfuise_walk1",
        "seed": specification["seed"],
        "prefix_identity": "full_original_crop",
        "config": str(config_path),
        "cells": make_cells(),
        "anchor_ids": anchor_ids,
    }
    manifest = {
        "schema": "uifgo_t09_batch_v2",
        "role": "development",
        "parameter_provenance":
            "IE0911_ORACLE_BACKEND_SANITY_DEBUG_TEST_ONLY_PENDING_VALIDATION",
        "run_units": [unit],
    }
    manifest_path = output / "batch_spec.yaml"
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    audit = {
        "schema": "uifgo_oracle_backend_preflight_v1",
        "baseline_head": BASELINE_HEAD,
        "actual_head": os.popen("git rev-parse HEAD").read().strip(),
        "regime": args.regime,
        "locked_input_identity": input_identity,
        "anchor_ids": anchor_ids,
        "support_sha256": sha256(support_path),
        "effective_source_config_sha256": sha256(config_path),
        "support_contains_true_amplitude": False,
        "estimator_config_contains_truth_or_gt_path": False,
        "fde_execution": "BYPASSED_FIXED_PARTITION_DEBUG",
    }
    if audit["actual_head"] != BASELINE_HEAD:
        raise RuntimeError("actual HEAD differs from registered oracle baseline")
    (output / "preflight_audit.json").write_text(
        json.dumps(audit, indent=2) + "\n", encoding="utf-8")

    trace = output / "estimator_file_access.trace"
    batch = output / "batch"
    command = [
        "bwrap", "--bind", "/", "/", "--dev-bind", "/dev", "/dev",
        "--tmpfs", str(LOCKED_ROOT / "truth"),
        "--tmpfs", "/home/mint/ws_fusion_uwb/res/ie0911_step2_truth_20260911_01",
        "--", "strace", "-f", "-e", "trace=%file", "-o", str(trace),
        "python3", str(common.ROOT / "tools/paper/run_experiments.py"),
        "--manifest", str(manifest_path), "--runner", str(common.BIN),
        "--output-root", str(batch),
    ]
    environment = os.environ.copy()
    environment["UIFGO_EXPERIMENT_WALL_LIMIT_S"] = "1800"
    code = common.execute(command, output / "batch_driver.log",
                          limit=1800 * len(unit["cells"]) + 120,
                          env=environment)
    (output / "execution.json").write_text(json.dumps({
        "command": command, "exit_code": code, "regime": args.regime,
        "no_algorithm_retry": True, "per_estimator_timeout_seconds": 1800,
    }, indent=2) + "\n", encoding="utf-8")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
