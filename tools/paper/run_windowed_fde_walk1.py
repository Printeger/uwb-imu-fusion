#!/usr/bin/env python3
"""Run the preregistered windowed-FDE v4 Walk1 gates and injected E2E."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil

import yaml

import nlos_experiment_common as common
import nlos_injection as injection
import nlos_injection_metrics as metrics


OLD = Path("/home/mint/ws_fusion_uwb/res/nlos_injection_20260911_01")
LOCKED_SHA = "sha256:1660f7a1e8d1bd32bfabda394adf9e69675fb849917b028b66fff7ec0e87e820"
PROVIDER = "imu_aided_windowed_fde_v4"
VERSION = "UIFGO_IMU_AIDED_WINDOWED_FDE_IDENTITY_V4"
RULE = "FDE_WINDOWED_DYADIC_BONFERRONI_MERGE_V4"
SUBSET = [7475, 9524, 10548, 15155, 20276]
LINK = [27956, 20276]
WINDOW = [1664959678.3077347, 1664959686.3077347]
EXPECTED = {
    "clean": {
        "input_manifest": "sha256:7af404c8580ecf0b9f9d2742bbc56d03ed9a592172603d3df0e8a0d569e22c0e",
        "truth_json": "sha256:ddf95c0c5663e9853771b6cc39ac90137b6191de51d2415cd8164121b55002e4",
        "truth_csv": "sha256:79bff8652d238fea551ed218204862e274a44c1921e7fe7ad4e835815411fe6b",
        "cache_id": "sha256:4330902d2f741459038f9072e3cc6f7ade2cb6fd1a19c1929679e7c8eb44bcfe",
        "transform_sha256": "sha256:43015644f04cb63db930f20f280e9dbf42badf98d92b6205d1db281ea107bc9b",
    },
    "injected": {
        "input_manifest": "sha256:fc3c37aa75a6141a90126cdc9ed14a7b4adace586aa091d0f601c419deee61d6",
        "truth_json": "sha256:d333bc3dbe93333779c8d22cd49e484fbbf2a453a65e3cbaede6d74480fe310a",
        "truth_csv": "sha256:97749b8b738b4dac3dd8bb4ed4e89a94e38b0295f478dbb6fbb4fc31f06eee78",
        "cache_id": "sha256:d967b14b27c45ca90cfcc3c357458efc082e6e6e3e3bd35168966942079b4f82",
        "transform_sha256": "sha256:679b4b582ee2529e817a2c5b862ac22835013c459f8cb83cf36a1bf76482c111",
    },
}


def load(path: Path) -> dict:
    return json.loads(path.read_text()) if path.is_file() else {}


def rows(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def sha(path: Path) -> str:
    return injection.sha(path.read_bytes())


def value(doc: dict, key: str):
    item = doc.get(key, {})
    return item.get("value") if item.get("status") == "AVAILABLE" else None


def write(path: Path, value_: object) -> None:
    injection.write_json(path, value_)


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise RuntimeError(reason)


def bwrap_prefix(trace: Path) -> list[str]:
    hidden = [OLD / "truth", common.ROOT / "data",
              Path("/home/mint/ws_fusion_uwb/res/ie0911_step2_truth_20260911_01")]
    command = ["bwrap", "--bind", "/", "/", "--dev-bind", "/dev", "/dev"]
    for path in hidden:
        if path.is_dir():
            command += ["--tmpfs", str(path)]
    return command + ["--", "strace", "-f", "-e", "trace=%file", "-o", str(trace)]


def access_audit(trace: Path) -> dict:
    text = trace.read_text(errors="replace") if trace.is_file() else ""
    tokens = ("nlos_injection_truth", "nlos_injected_observations",
              "ground_truth.tum", "ground_truth.csv")
    forbidden = [line for line in text.splitlines()
                 if ("open(" in line or "openat(" in line) and
                 any(token in line for token in tokens)]
    return {"trace": str(trace), "trace_sha256": sha(trace),
            "forbidden_opens": forbidden, "hidden_truth_directories": True,
            "pass": not forbidden}


def preflight(root: Path, locked: dict) -> dict:
    require(sha(OLD / "locked_manifest.json") == LOCKED_SHA,
            "LOCKED_MANIFEST_HASH_MISMATCH")
    scenarios = {}
    audit = {"locked_manifest_sha256": LOCKED_SHA, "scenarios": {}}
    for condition in ("clean", "injected"):
        scenario_id = f"sfuise_walk1_normal_{condition}"
        scenario = next((item for item in locked["scenarios"]
                         if item.get("scenario_id") == scenario_id), None)
        require(scenario is not None, "LOCKED_SCENARIO_MISSING")
        require(scenario["subset"] == SUBSET and scenario["link"] == LINK and
                scenario["window"] == WINDOW, "LOCKED_SELECTION_MISMATCH")
        truth_dir = Path(scenario["truth"])
        config_path = Path(scenario["config"])
        config = yaml.safe_load(config_path.read_text())
        manifest_path = Path(config["dataset"]["cache_manifest"])
        manifest = load(manifest_path)
        truth = load(truth_dir / "nlos_injection_truth.json")
        expected = EXPECTED[condition]
        require(sha(manifest_path) == expected["input_manifest"],
                "INPUT_MANIFEST_HASH_MISMATCH")
        require(sha(truth_dir / "nlos_injection_truth.json") == expected["truth_json"] and
                sha(truth_dir / "nlos_injected_observations.csv") == expected["truth_csv"],
                "TRUTH_SIDECAR_HASH_MISMATCH")
        require(manifest.get("cache_id") == expected["cache_id"] and
                manifest.get("transform_sha256") == expected["transform_sha256"] and
                truth.get("transform_sha256") == expected["transform_sha256"],
                "INPUT_TRUTH_IDENTITY_MISMATCH")
        require(sha(manifest_path.parent / manifest["imu_file"]) == manifest["imu_sha256"] and
                sha(manifest_path.parent / manifest["uwb_file"]) == manifest["uwb_sha256"],
                "INPUT_PAYLOAD_HASH_MISMATCH")
        specification = truth["specification"]
        require(specification["anchor_subset"] == SUBSET and
                [specification["tag_id"], specification["anchor_id"]] == LINK and
                [specification["start_s"], specification["end_s"]] == WINDOW and
                specification["seed"] == 911 and specification["amplitude_m"] == 0.5 and
                specification["interval"] == "closed",
                "INJECTION_SPECIFICATION_MISMATCH")
        if condition == "clean":
            require(not specification["enabled"] and not truth["affected_obs_ids"] and
                    not truth["planned_affected_obs_ids"], "CLEAN_TRUTH_NOT_EMPTY")
        else:
            require(specification["enabled"] and len(truth["affected_obs_ids"]) == 111 and
                    len(truth["planned_affected_obs_ids"]) == 30,
                    "INJECTION_COUNTS_MISMATCH")
        config.setdefault("nlos", {})["fde_grouped_test"] = False
        config["nlos"]["fde_windowed_test"] = True
        effective_path = root / "configs" / f"walk1_{condition}_v4.yaml"
        effective_path.parent.mkdir(parents=True, exist_ok=True)
        effective_path.write_text(yaml.safe_dump(config, sort_keys=True))
        scenarios[condition] = (scenario, effective_path)
        audit["scenarios"][condition] = {
            "scenario_id": scenario_id, "config_source": str(config_path),
            "effective_config": str(effective_path), "effective_config_sha256": sha(effective_path),
            "input_manifest": str(manifest_path), "input_manifest_sha256": sha(manifest_path),
            "input_cache_id": manifest["cache_id"], "truth_directory": str(truth_dir),
            "truth_json_sha256": sha(truth_dir / "nlos_injection_truth.json"),
            "truth_csv_sha256": sha(truth_dir / "nlos_injected_observations.csv"),
            "affected_raw_count": len(truth["affected_obs_ids"]),
            "affected_planned_count": len(truth["planned_affected_obs_ids"]),
        }
    write(root / "locked_input_audit.json", audit)
    return scenarios


def detector_screen(root: Path, condition: str, config: Path,
                    scenario: dict) -> dict:
    output = root / "detector_screens" / condition
    output.mkdir(parents=True, exist_ok=False)
    trace = root / "detector_screens" / f"{condition}_file_access.trace"
    run_id = f"walk1_{condition}_detector_v4"
    command = bwrap_prefix(trace) + [
        str(common.BIN), "--config", str(config), "--output-root", str(output),
        "--run-id", run_id, "--stop-after-fde", "--execution-type",
        "CACHE_PRODUCER", "--method", "structured_debias", "--anchor-ids",
        ",".join(map(str, SUBSET)),
    ]
    code = common.execute(command, root / "detector_screens" / f"{condition}.log",
                          limit=1800)
    run = output / run_id
    status = load(run / "fde_status.json")
    partition = load(run / "support_partition.json")
    records = rows(run / "fde_observations.csv")
    artifact_names = ("fde_local_windows.csv", "fde_windowed_summary.json",
                      "fde_observations.csv", "support_partition.json", "partition.json")
    artifacts = {name: sha(run / name) for name in artifact_names
                 if (run / name).is_file()}
    audit = access_audit(trace)
    require(audit["pass"], "DETECTOR_TRUTH_OPENED")
    complete = (code == 0 and status.get("status") == "SUCCESS" and
                status.get("provider") == PROVIDER and
                status.get("provider_version") == VERSION and
                partition.get("partition_rule_version") == RULE and
                status.get("planned_count") == 1074 and
                status.get("tested_count") == status.get("planned_count") and
                set(artifact_names).issubset(artifacts))
    truth = load(Path(scenario["truth"]) / "nlos_injection_truth.json")
    planned = {row["obs_id"] for row in records
               if row["valid"] == "1" and row["planned"] == "1"}
    detection = metrics.detection(records, set(truth["affected_obs_ids"]),
                                  WINDOW[0], complete, planned)
    segment_metrics = []
    for segment in partition.get("segments", []):
        selected = [row for row in records
                    if row.get("segment_id") == segment["segment_id"]]
        sm = metrics.segment_metric(
            selected, set(truth["affected_obs_ids"]) & planned, *WINDOW)
        overlap = sm.get("interval_overlap_s") if segment["tag_id"] == LINK[0] and segment["anchor_id"] == LINK[1] else 0.0
        start = float(segment["start_time"])
        end = float(segment["end_time"])
        union = max(end, WINDOW[1]) - min(start, WINDOW[0])
        segment_metrics.append({
            "segment_id": segment["segment_id"], "tag_id": segment["tag_id"],
            "anchor_id": segment["anchor_id"], **sm,
            "temporal_overlap_s": overlap,
            "temporal_iou": overlap / union if union > 0 else None,
            "start_error_s": start - WINDOW[0], "end_error_s": end - WINDOW[1],
        })
    retained = status.get("retained_segment_count")
    clean_gate = complete and retained == 0
    injected_overlap = any(item["temporal_overlap_s"] > 0 for item in segment_metrics)
    injected_gate = complete and retained and injected_overlap
    return {
        "condition": condition, "exit_code": code, "run_directory": str(run),
        "complete": complete, "fde": status, "partition": partition,
        "artifacts": artifacts, "access_audit": audit, "detection": detection,
        "segment_metrics": segment_metrics,
        "gate_passed": bool(clean_gate if condition == "clean" else injected_gate),
        "gate_failure": (None if (clean_gate if condition == "clean" else injected_gate)
                         else ("DETECTOR_GATE_FAIL_CLEAN_FALSE_SUPPORT" if condition == "clean"
                               else "DETECTOR_GATE_FAIL_MISSED_INJECTION")),
    }


def run_batch(root: Path, config: Path) -> tuple[int, Path, dict]:
    methods = list(injection.METHODS)
    unit = common.manifest_unit("sfuise_walk1", "walk1_injected_v4", config,
                                SUBSET, modes=methods, producer=True)
    specification = root / "walk1_injected_six_methods.yaml"
    specification.write_text(yaml.safe_dump({
        "schema": "uifgo_t09_batch_v2", "role": "development",
        "parameter_provenance": "WINDOWED_FDE_V4_LOCKED_WALK1_DEVELOPMENT_ONLY",
        "run_units": [unit],
    }, sort_keys=False))
    output = root / "injected_six_methods"
    trace = root / "injected_six_methods_file_access.trace"
    command = bwrap_prefix(trace) + [
        "python3", str(common.ROOT / "tools/paper/run_experiments.py"),
        "--manifest", str(specification), "--runner", str(common.BIN),
        "--output-root", str(output),
    ]
    env = os.environ.copy()
    env["UIFGO_EXPERIMENT_WALL_LIMIT_S"] = "1800"
    code = common.execute(command, root / "injected_six_methods.log",
                          limit=1800 * 8 + 120, env=env)
    audit = access_audit(trace)
    require(audit["pass"], "E2E_ESTIMATOR_TRUTH_OPENED")
    return code, output, audit


def evaluate_batch(root: Path, output: Path, screen: dict,
                   scenario: dict, code: int, access: dict) -> dict:
    batch = load(output / "batch_manifest.json")
    cells = batch.get("cells", [])
    producer = next((item for item in cells
                     if item.get("execution_type") == "CACHE_PRODUCER"), {})
    producer_run = Path(producer["run_directory"]) if producer.get("run_directory") else None
    producer_partition = load(producer_run / "support_partition.json") if producer_run else {}
    producer_fde = load(producer_run / "fde_status.json") if producer_run else {}
    producer_context = load(producer_run / "stage2_producer_context.json") if producer_run else {}
    stage2_status = load(producer_run / "stage2_refit_status.json") if producer_run else {}
    screen_ids = {obs for seg in screen["partition"].get("segments", [])
                  for obs in seg.get("obs_ids", [])}
    producer_ids = {obs for seg in producer_partition.get("segments", [])
                    for obs in seg.get("obs_ids", [])}
    support_consistency = {
        "provider_equal": producer_partition.get("provider") == screen["partition"].get("provider") == PROVIDER,
        "partition_hash_equal": producer_partition.get("partition_hash") == screen["partition"].get("partition_hash"),
        "observation_union_equal": producer_ids == screen_ids,
        "screen_observation_union": sorted(screen_ids, key=int),
        "producer_observation_union": sorted(producer_ids, key=int),
        "support_source": producer_context.get("stage1_provider", producer_fde.get("provider")),
        "stage2_status": stage2_status,
    }
    support_consistency["pass"] = (
        all(support_consistency[key] for key in
            ("provider_equal", "partition_hash_equal", "observation_union_equal")) and
        support_consistency["support_source"] == PROVIDER and
        producer.get("status") in {"COMPLETE", "COMPLETE_WITH_SCORE_UNAVAILABLE"})

    protocol = yaml.safe_load(
        (common.ROOT / "config/paper/ie0911/step2_evaluation.yaml").read_text()
    )["run_units"]["sfuise_walk1"]
    require(sha(Path(protocol["ground_truth"])) == protocol["ground_truth_sha256"],
            "LOCALIZATION_GROUND_TRUTH_HASH_MISMATCH")
    finals = {}
    for method in injection.METHODS:
        cell = next((item for item in cells
                     if item.get("canonical_mode") == method and
                     item.get("execution_type") != "CACHE_PRODUCER"), {})
        run = Path(cell["run_directory"]) if cell.get("run_directory") else None
        metric = metrics.localization(run, protocol, tuple(WINDOW)) \
            if run and cell.get("status") == "COMPLETE" else {}
        finals[method] = {
            "cell_status": cell.get("status", "NOT_RUN"),
            "reason": cell.get("reason"), "run_directory": str(run) if run else None,
            "cache_id": cell.get("cache_id"),
            "aligned_ATE_rmse_m": value(metric, "aligned_ATE_rmse_m"),
            "aligned_ATE_p95_m": value(metric, "aligned_ATE_p95_m"),
            "horizontal_rmse_m": value(metric, "aligned_horizontal_rmse_m"),
            "vertical_rmse_m": value(metric, "aligned_height_rmse_m"),
            "coverage": metric.get("trajectory_coverage"),
            "window": metric.get("window", {"status": "UNAVAILABLE"}),
            "run_status": load(run / "run_status.json") if run else {},
            "covariance": load(run / "covariance_status.json") if run else {},
            "fallback": load(run / "fallback_attempt.json") if run else {},
            "fixed_compensations": rows(run / "fixed_compensations.csv") if run else [],
            "inference_identity": load(run / "inference_identity.json") if run else {},
        }
    cache_ids = {finals[name]["cache_id"] for name in
                 ("suppress_all", "structured_debias", "lcb_fixed_full", "lcb_partial")
                 if finals[name]["cache_id"]}
    stage2_values = load(producer_run / "stage2_content_identity.json") if producer_run else {}
    rmse = {name: finals[name]["aligned_ATE_rmse_m"] for name in finals}
    differences = {}
    for name in ("lcb_fixed_full", "lcb_partial"):
        differences[name] = {
            "minus_suppress_all_m": (rmse[name] - rmse["suppress_all"]
                                     if rmse[name] is not None and rmse["suppress_all"] is not None else None),
            "minus_injected_raw_m": (rmse[name] - rmse["all_range"]
                                     if rmse[name] is not None and rmse["all_range"] is not None else None),
        }
    evaluable = all(rmse[name] is not None for name in
                    ("suppress_all", "lcb_fixed_full", "lcb_partial"))
    localization = ("NOT_EVALUABLE" if not evaluable else
                    "BENEFIT_OBSERVED" if min(rmse["lcb_fixed_full"], rmse["lcb_partial"]) < rmse["suppress_all"]
                    else "NO_LOCALIZATION_BENEFIT")
    lcb_vs_suppression = (
        "LCB_VS_SUPPRESSION_BENEFIT_OBSERVED" if evaluable and
        rmse["lcb_partial"] < rmse["suppress_all"] else None)
    detector_pass = screen["gate_passed"] and support_consistency["pass"]
    return {
        "batch_exit_code": code, "batch_status": batch.get("status"),
        "batch_directory": str(output), "access_audit": access,
        "producer": producer, "producer_run": str(producer_run) if producer_run else None,
        "producer_fde": producer_fde, "producer_partition": producer_partition,
        "support_consistency": support_consistency,
        "shared_candidate_cache_ids": sorted(cache_ids),
        "shared_candidate_cache_pass": len(cache_ids) == 1,
        "stage2_values_identity": stage2_values, "finals": finals,
        "rmse_differences": differences,
        "detector_verdict": "PASS" if detector_pass else "FAIL",
        "localization_verdict": localization,
        "lcb_vs_suppression_verdict": lcb_vs_suppression,
        "larger_matrix": "NOT_RUN_TASK_SCOPE",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.output.resolve()
    require(root.is_dir(), "RESERVED_OUTPUT_ROOT_MISSING")
    execution = root / "walk1_execution.json"
    require(not execution.exists(), "REFUSING_TO_OVERWRITE_WALK1_EXECUTION")
    require(load(root / "engineering/final/engineering_gate.json").get("status") == "PASS",
            "ENGINEERING_GATE_NOT_PASSED")
    locked = load(OLD / "locked_manifest.json")
    scenarios = preflight(root, locked)
    report = {
        "schema": "uifgo_windowed_fde_walk1_v1", "provider": PROVIDER,
        "provider_version": VERSION, "partition_rule": RULE,
        "engineering_gate": "PASS", "screens": {},
        "detector_verdict": "PENDING", "localization_verdict": "NOT_RUN",
        "walk2_walk3_low_redundancy": "NOT_RUN_TASK_SCOPE",
    }
    for condition in ("clean", "injected"):
        scenario, config = scenarios[condition]
        result = detector_screen(root, condition, config, scenario)
        report["screens"][condition] = result
        write(execution, report)
        if not result["gate_passed"]:
            report["detector_verdict"] = "FAIL"
            report["stop_reason"] = result["gate_failure"]
            report["localization_verdict"] = "NOT_EVALUABLE"
            write(execution, report)
            return 0

    code, batch_output, access = run_batch(root, scenarios["injected"][1])
    e2e = evaluate_batch(root, batch_output, report["screens"]["injected"],
                         scenarios["injected"][0], code, access)
    report["e2e"] = e2e
    report["detector_verdict"] = e2e["detector_verdict"]
    report["localization_verdict"] = e2e["localization_verdict"]
    report["lcb_vs_suppression_verdict"] = e2e["lcb_vs_suppression_verdict"]
    report["next_round"] = ("ALLOWED_BUT_NOT_RUN" if
                            e2e["detector_verdict"] == "PASS" and
                            e2e["localization_verdict"] != "NOT_EVALUABLE"
                            else "NOT_ALLOWED")
    write(execution, report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
