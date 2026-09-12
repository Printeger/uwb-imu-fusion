#!/usr/bin/env python3
"""Evaluate SFUISE and four frozen methods under one trajectory protocol."""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import uuid


ROOT = Path(__file__).resolve().parents[2]
RES = ROOT.parents[1] / "res/nlos_injection_20260911_01"
GT_ROOT = ROOT.parents[1] / "res/ie0911_step2_truth_20260911_01"
EVALUATOR = ROOT / "tools/paper/evaluate_runs.py"
METHODS = (
    ("Base FGO", "all_range", "baselines"),
    ("Robust FGO", "robust_cauchy", "baselines"),
    ("SFUISE", "SFUISE", "sfuise"),
    ("suppress_all", "suppress_all", "e2e"),
    ("lcb_fixed_full", "lcb_fixed_full", "e2e"),
)


def read_json(path: Path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def sha256(path: Path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return "sha256:" + digest.hexdigest()


def load_evaluator():
    spec = importlib.util.spec_from_file_location("icra_unified_evaluator", EVALUATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load unified evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def interval(sequence: int):
    path = RES / f"inputs/sfuise_walk{sequence}_normal_clean/uwb_observations.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        times = [float(row["sensor_time_s"]) for row in csv.DictReader(stream)]
    if not times or not all(math.isfinite(value) for value in times):
        raise RuntimeError(f"invalid UWB time basis: {path}")
    return min(times), max(times), path


def find_run(sequence: int, mode: str, kind: str):
    batch = RES / f"batches/sfuise_walk{sequence}_normal_clean_{kind}"
    matches = []
    for manifest_path in sorted(batch.glob("runs/*/run_manifest.json")):
        manifest = read_json(manifest_path)
        if manifest.get("canonical_mode") == mode and manifest.get("execution_type") in {
                "BASELINE_TRAJECTORY", "FINAL_TRAJECTORY"}:
            matches.append((manifest_path.parent, manifest))
    if len(matches) != 1:
        raise RuntimeError(f"expected one sealed {mode} run for Walk{sequence}, got {len(matches)}")
    return matches[0]


def validate_and_clip(source: Path, destination: Path, start: float, end: float):
    kept = []
    previous = None
    with source.open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            values = [float(value) for value in line.split()]
            if len(values) != 8 or not all(math.isfinite(value) for value in values):
                raise ValueError(f"invalid trajectory row {source}:{number}")
            if previous is not None and values[0] <= previous:
                raise ValueError(f"non-increasing trajectory timestamp {source}:{number}")
            if not 0.99 <= sum(value * value for value in values[4:8]) <= 1.01:
                raise ValueError(f"non-unit trajectory quaternion {source}:{number}")
            previous = values[0]
            if start <= values[0] <= end:
                kept.append(" ".join(f"{value:.17g}" for value in values) + "\n")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("".join(kept), encoding="utf-8")
    return len(kept)


def metric_value(metrics, name):
    cell = metrics.get(name, {})
    return cell.get("value") if cell.get("status") == "AVAILABLE" else None


def write_csv(path: Path, rows, fieldnames):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sfuise-batch", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, default=ROOT / "experiments/results")
    args = parser.parse_args()
    batch = read_json(args.sfuise_batch / "batch_manifest.json")
    sfuise_runs = {int(run["sequence"][len("ISAS-Walk"):]): run
                   for run in batch["runs"]}
    output = args.output_root.resolve() / (
        "sfuise-comparison-" + dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        + "-" + uuid.uuid4().hex[:12])
    output.mkdir(parents=True)
    evaluator = load_evaluator()
    protocol = {
        "schema": "sfuise_toa_comparison_protocol_v1",
        "dataset_family": "SFUISE/ISAS",
        "measurement_type": "ABSOLUTE_TOA_RANGE",
        "time_association": {"policy": "nearest_within_tolerance", "tolerance_s": 0.02},
        "alignment_mode": "SE3_SCALE_1_PER_TRAJECTORY",
        "reference_point": "tracker_origin_assumed_body_imu_origin",
        "T_G_E": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
        "T_Q_I": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
        "frame_point_provenance": "ISAS tracker/body co-location developmental assumption; identical for all methods",
        "rpe_horizon_s": 1.0,
        "evaluator": str(EVALUATOR),
        "evaluator_sha256": sha256(EVALUATOR),
    }
    long_rows = []
    full_metrics = []
    for sequence in (1, 2, 3):
        start, end, uwb_path = interval(sequence)
        gt = GT_ROOT / f"sfuise_walk{sequence}/ground_truth.tum"
        if not gt.is_file():
            raise RuntimeError(f"GT unavailable: {gt}")
        scenario = dict(protocol, ground_truth=str(gt), evaluation_interval_s=[start, end])
        for display, mode, kind in METHODS:
            if kind == "sfuise":
                run = sfuise_runs[sequence]
                source_dir = Path(run.get("trajectory_path", "")).parent
                source = Path(run.get("trajectory_path", ""))
                status = run.get("status", "FAILURE")
                source_detail = run.get("failure_reason", "")
                runtime = run.get("wall_time_s")
                runtime_semantics = "END_TO_END_WALL_INCLUDING_1X_PLAYBACK"
                run_id = run.get("run_id")
                source_status = args.sfuise_batch / f"ISAS-Walk{sequence}/run_status.json"
            else:
                source_dir, manifest = find_run(sequence, mode, kind)
                state = read_json(source_dir / "run_status.json")
                source = source_dir / "trajectory.tum"
                status = state.get("status", "FAILURE")
                source_detail = state.get("reason", "")
                runtime = state.get("elapsed_seconds")
                runtime_semantics = "SEALED_ESTIMATOR_ELAPSED_SECONDS"
                run_id = manifest["run_id"]
                source_status = source_dir / "run_status.json"
            available = status in {"SUCCESS", "OK", "NO_CANDIDATES"} and source.is_file()
            eval_dir = output / "evaluator_inputs" / f"Walk{sequence}" / mode
            eval_dir.mkdir(parents=True, exist_ok=True)
            trajectory_rows = 0
            if available:
                trajectory_rows = validate_and_clip(source, eval_dir / "trajectory.tum", start, end)
            # Failed sources also enter the same evaluator; its missing-trajectory
            # contract produces explicit unavailable metrics instead of a fake row.
            metrics = evaluator.trajectory_metrics(eval_dir, scenario)
            ate = metric_value(metrics, "aligned_ATE_rmse_m")
            matched = metric_value(metrics, "matched_count")
            eval_status = "SUCCESS" if ate is not None else "FAILURE"
            reason = ""
            if not available:
                eval_status = "FAILURE"
                reason = source_detail or f"SOURCE_STATUS_{status}"
            elif ate is None:
                reason = metrics.get("aligned_ATE_rmse_m", {}).get("reason", "ATE_UNAVAILABLE")
            row = {
                "sequence": f"ISAS Walk{sequence}", "method": display, "run_id": run_id,
                "source_status": status, "evaluation_status": eval_status,
                "source_detail": source_detail, "failure_reason": reason, "ate_rmse_m": ate,
                "evaluation_samples": matched or 0, "runtime_s": runtime,
                "runtime_semantics": runtime_semantics,
                "trajectory_rows_in_interval": trajectory_rows,
                "trajectory_path": str(source) if source else "",
                "trajectory_sha256": sha256(source) if source.is_file() else "",
                "source_status_path": str(source_status), "source_status_sha256": sha256(source_status),
                "evaluation_start_s": start, "evaluation_end_s": end,
                "gt_path": str(gt), "gt_sha256": sha256(gt),
                "time_association": "nearest_within_tolerance_0.02s",
                "alignment_mode": protocol["alignment_mode"],
                "reference_point": protocol["reference_point"],
                "measurement_type": protocol["measurement_type"],
            }
            long_rows.append(row)
            full_metrics.append({"sequence": sequence, "method": display, "run_id": run_id,
                                 "scenario": scenario, "metrics": metrics})
    fields = list(long_rows[0])
    write_csv(output / "baseline_runs.csv", long_rows, fields)
    write_csv(output / "trajectory_metrics.csv", long_rows, fields)
    table = []
    for sequence in (1, 2, 3):
        selected = {row["method"]: row for row in long_rows if row["sequence"] == f"ISAS Walk{sequence}"}
        table.append({"Sequence": f"ISAS Walk{sequence}", **{
            display: selected[display]["ate_rmse_m"] if selected[display]["ate_rmse_m"] is not None else "NA"
            for display, _, _ in METHODS}})
    write_csv(output / "main_table.csv", table, ["Sequence"] + [item[0] for item in METHODS])
    manifest = {"schema": "sfuise_toa_comparison_v1", "status": "COMPLETE_WITH_RETAINED_FAILURES",
                "output": str(output), "sfuise_batch": str(args.sfuise_batch.resolve()),
                "protocol": protocol, "protocol_sha256": "sha256:" + hashlib.sha256(
                    json.dumps(protocol, sort_keys=True, separators=(",", ":")).encode()).hexdigest(),
                "metrics": full_metrics}
    (output / "comparison_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    shutil.rmtree(output / "evaluator_inputs")
    print(json.dumps({"status": manifest["status"], "output": str(output)}))


if __name__ == "__main__":
    main()
