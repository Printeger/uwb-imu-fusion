#!/usr/bin/env python3
"""Analyze compact five-profile raw-stream runs without using production math."""

import argparse
import csv
import hashlib
import json
import math
import re
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Optional


def rows(path: Path) -> List[dict]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def number(value: str) -> Optional[float]:
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def percentile(values: Iterable[float], probability: float) -> Optional[float]:
    data = sorted(x for x in values if math.isfinite(x))
    if not data:
        return None
    index = max(0, math.ceil(probability * len(data)) - 1)
    return data[min(index, len(data) - 1)]


def rms(values: Iterable[float]) -> Optional[float]:
    data = [x for x in values if math.isfinite(x)]
    return math.sqrt(sum(x*x for x in data) / len(data)) if data else None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def quaternion_error(a: dict, b: dict) -> float:
    dot = abs(sum(float(a[key]) * float(b[key])
                  for key in ("qw", "qx", "qy", "qz")))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def vector_error(a: dict, b: dict, keys) -> float:
    return math.sqrt(sum((float(a[x]) - float(b[y]))**2 for x, y in keys))


def truth_velocities(truth: List[dict]) -> Dict[int, tuple]:
    result: Dict[int, tuple] = {}
    for index, row in enumerate(truth):
        before = truth[max(0, index - 1)]
        after = truth[min(len(truth) - 1, index + 1)]
        dt = (int(after["timestamp_ns"]) - int(before["timestamp_ns"])) * 1e-9
        if dt > 0:
            result[int(row["timestamp_ns"])] = tuple(
                (float(after[key]) - float(before[key])) / dt
                for key in ("px", "py", "pz"))
    return result


def run_identity(run: Path) -> dict:
    metadata = run / "validation_run.json"
    if metadata.exists():
        return json.loads(metadata.read_text(encoding="utf-8"))
    manifest = run / "run_manifest.json"
    return {"run_id": run.name, "profile": "unknown", "scenario": "unknown",
            "manifest_present": manifest.exists()}


def analyze_run(run: Path) -> dict:
    state_rows = rows(run / "states.csv")
    truth_rows = rows(run / "ground_truth.csv")
    integrity_rows = rows(run / "integrity.csv")
    timing_rows = rows(run / "timing.csv")
    attempt_rows = rows(run / "diagnostic_attempts.csv")
    fault_rows = rows(run / "fault_truth.csv")
    identity = run_identity(run)
    truth_by_time = {int(row["timestamp_ns"]): row for row in truth_rows}
    velocity_by_time = truth_velocities(truth_rows)
    position_errors, attitude_errors, velocity_errors = [], [], []
    accel_bias_errors, gyro_bias_errors = [], []
    hpl_values, vpl_values = [], []
    finite_pl_with_truth = 0
    horizontal_covered = 0
    vertical_covered = 0
    xyz_covered = 0
    service_position_errors = []
    for index, state in enumerate(state_rows):
        timestamp = int(state["timestamp_ns"])
        truth = truth_by_time.get(timestamp)
        if truth:
            axis_errors = tuple(float(state[key]) - float(truth[key])
                                for key in ("px", "py", "pz"))
            position_errors.append(math.sqrt(sum(x*x for x in axis_errors)))
            attitude_errors.append(quaternion_error(state, truth))
            accel_bias_errors.append(math.sqrt(sum(float(state[key])**2
                                                     for key in ("bax", "bay", "baz"))))
            gyro_bias_errors.append(math.sqrt(sum(float(state[key])**2
                                                    for key in ("bgx", "bgy", "bgz"))))
            if timestamp in velocity_by_time:
                target = velocity_by_time[timestamp]
                velocity_errors.append(math.sqrt(sum(
                    (float(state[key]) - target[axis])**2
                    for axis, key in enumerate(("vx", "vy", "vz")))))
            if index < len(integrity_rows):
                integrity = integrity_rows[index]
                hpl = number(integrity.get("hpl_m"))
                vpl = number(integrity.get("vpl_m"))
                pls = [number(integrity.get(key))
                       for key in ("pl_x", "pl_y", "pl_z")]
                if (integrity.get("pl_status") == "FINITE" and
                        hpl is not None and vpl is not None and
                        all(value is not None for value in pls)):
                    finite_pl_with_truth += 1
                    horizontal_covered += math.hypot(*axis_errors[:2]) <= hpl
                    vertical_covered += abs(axis_errors[2]) <= vpl
                    xyz_covered += all(abs(error) <= bound
                                       for error, bound in zip(axis_errors, pls))
        if index < len(integrity_rows):
            attempted = int(integrity_rows[index].get("attempted_timestamp_ns", 0))
            attempted_truth = truth_by_time.get(attempted)
            if attempted_truth:
                service_position_errors.append(vector_error(
                    state, attempted_truth,
                    (("px", "px"), ("py", "py"), ("pz", "pz"))))

    core = [number(row["wall_ms"]) for row in timing_rows
            if row.get("stage") == "core_total"]
    outer = [number(row["wall_ms"]) for row in timing_rows
             if row.get("stage") == "outer_epoch"]
    core = [x for x in core if x is not None]
    outer = [x for x in outer if x is not None]
    warm_core = core[100:] if len(core) > 100 else core
    committed = sum(row.get("batch_committed") in ("1", "true", "True")
                    for row in integrity_rows)
    fresh = sum(row.get("fresh") in ("1", "true", "True")
                for row in integrity_rows)
    deadline = sum(row.get("deadline_missed") in ("1", "true", "True")
                   for row in integrity_rows)
    finite = sum(row.get("pl_status") == "FINITE" for row in integrity_rows)
    within = sum(row.get("within_alert_limits") in ("1", "true", "True")
                 for row in integrity_rows)
    for row in integrity_rows:
        if row.get("pl_status") == "FINITE":
            hpl = number(row.get("hpl_m"))
            vpl = number(row.get("vpl_m"))
            if hpl is not None: hpl_values.append(hpl)
            if vpl is not None: vpl_values.append(vpl)
    stage_values: Dict[str, List[float]] = {}
    for row in timing_rows:
        value = number(row.get("wall_ms"))
        if value is not None:
            stage_values.setdefault(row.get("stage", "unknown"), []).append(value)
    stage_summary = {
        stage: {"count": len(values), "p50_ms": percentile(values, .50),
                "p95_ms": percentile(values, .95),
                "p99_ms": percentile(values, .99), "max_ms": max(values)}
        for stage, values in sorted(stage_values.items())
    }
    hashes = {}
    for name in ("resolved_config.yaml", "run_manifest.json", "states.csv",
                 "ground_truth.csv", "integrity.csv", "diagnostic_attempts.csv",
                 "diagnostic_stages.csv", "fault_truth.csv", "timing.csv",
                 "summary.json", "stdout.log", "stderr.log"):
        path = run / name
        if path.exists():
            hashes[name] = {"sha256": sha256(path), "bytes": path.stat().st_size}
    count = len(integrity_rows)
    marginalizations = max((int(float(row.get("marginalization_count", 0) or 0))
                            for row in attempt_rows), default=0)
    fault_times = sorted({int(row["timestamp_ns"]) for row in fault_rows
                          if row.get("active") in ("1", "true", "True")})
    phases = {name: [] for name in ("pre_fault", "fault_active", "post_fault")}
    if fault_times:
        first_fault, last_fault = fault_times[0], fault_times[-1]
        active_times = set(fault_times)
        for row in integrity_rows:
            timestamp = int(row.get("attempted_timestamp_ns", row["timestamp_ns"]))
            phase = ("fault_active" if timestamp in active_times else
                     "pre_fault" if timestamp < first_fault else "post_fault")
            phases[phase].append(row)
    def phase_summary(items):
        total = len(items)
        finite_count = sum(row.get("pl_status") == "FINITE" for row in items)
        within_count = sum(row.get("within_alert_limits") in ("1", "true", "True")
                           for row in items)
        phase_hpl = [number(row.get("hpl_m")) for row in items
                     if row.get("pl_status") == "FINITE"]
        phase_vpl = [number(row.get("vpl_m")) for row in items
                     if row.get("pl_status") == "FINITE"]
        phase_hpl = [value for value in phase_hpl if value is not None]
        phase_vpl = [value for value in phase_vpl if value is not None]
        return {"epochs": total,
                "committed": sum(row.get("batch_committed") in ("1", "true", "True")
                                 for row in items),
                "finite_pl": finite_count,
                "finite_pl_fraction": finite_count / total if total else None,
                "within_alert_fraction": within_count / total if total else None,
                "hpl_p50_m": percentile(phase_hpl, .50),
                "hpl_p95_m": percentile(phase_hpl, .95),
                "hpl_max_m": max(phase_hpl) if phase_hpl else None,
                "vpl_p50_m": percentile(phase_vpl, .50),
                "vpl_p95_m": percentile(phase_vpl, .95),
                "vpl_max_m": max(phase_vpl) if phase_vpl else None,
                "fde_status_counts": dict(Counter(row.get("fde_status") for row in items)),
                "selected_action_counts": dict(Counter(
                    row.get("selected_action_type") for row in items))}
    rss_kib = None
    stderr_path = run / "stderr.log"
    if stderr_path.exists():
        match = re.search(r"Maximum resident set size \(kbytes\):\s*(\d+)",
                          stderr_path.read_text(encoding="utf-8", errors="replace"))
        if match: rss_kib = int(match.group(1))
    census_fields = ("single_uwb_hypotheses", "single_accel_hypotheses",
                     "single_gyro_hypotheses", "double_uwb_accel_hypotheses",
                     "double_uwb_gyro_hypotheses", "generated_actions")
    census = {field: max((int(float(row.get(field, 0) or 0)) for row in attempt_rows),
                         default=0) for field in census_fields}
    result = {
        **identity,
        "artifact_directory": str(run.resolve()),
        "counts": {"attempted": count, "committed": committed,
                   "rejected": count - committed, "fresh": fresh,
                   "deadline_missed": deadline, "finite_pl": finite,
                   "within_alert_limits": within,
                   "marginalizations": marginalizations},
        "accuracy": {
            "truth_at_state_position_rmse_m": rms(position_errors),
            "truth_at_state_position_p95_m": percentile(position_errors, .95),
            "truth_at_state_position_max_m": max(position_errors) if position_errors else None,
            "truth_at_attempt_position_rmse_m": rms(service_position_errors),
            "attitude_geodesic_rmse_rad": rms(attitude_errors),
            "attitude_geodesic_max_rad": max(attitude_errors) if attitude_errors else None,
            "velocity_rmse_mps": rms(velocity_errors),
            "accel_bias_norm_rmse_mps2": rms(accel_bias_errors),
            "gyro_bias_norm_rmse_radps": rms(gyro_bias_errors),
            "accel_bias_truth": [0.0, 0.0, 0.0],
            "gyro_bias_truth": [0.0, 0.0, 0.0],
        },
        "integrity": {
            "finite_pl_fraction": finite / count if count else None,
            "within_alert_fraction": within / count if count else None,
            "fresh_fraction": fresh / count if count else None,
            "deadline_miss_fraction": deadline / count if count else None,
        },
        "protection_level": {
            "finite_with_truth": finite_pl_with_truth,
            "hpl_p50_m": percentile(hpl_values, .50),
            "hpl_p95_m": percentile(hpl_values, .95),
            "hpl_max_m": max(hpl_values) if hpl_values else None,
            "vpl_p50_m": percentile(vpl_values, .50),
            "vpl_p95_m": percentile(vpl_values, .95),
            "vpl_max_m": max(vpl_values) if vpl_values else None,
            "horizontal_error_coverage_fraction": (
                horizontal_covered / finite_pl_with_truth
                if finite_pl_with_truth else None),
            "vertical_error_coverage_fraction": (
                vertical_covered / finite_pl_with_truth
                if finite_pl_with_truth else None),
            "axis_error_coverage_fraction": (
                xyz_covered / finite_pl_with_truth
                if finite_pl_with_truth else None),
        },
        "fault_phases": ({name: phase_summary(items) for name, items in phases.items()}
                         if fault_times else {}),
        "action_and_status_counts": {
            "fde_status": dict(Counter(row.get("fde_status") for row in integrity_rows)),
            "selected_action": dict(Counter(
                row.get("selected_action_type") for row in integrity_rows)),
        },
        "hypothesis_census_max": census,
        "performance": {
            "core_p50_ms": percentile(warm_core, .50),
            "core_p95_ms": percentile(warm_core, .95),
            "core_p99_ms": percentile(warm_core, .99),
            "core_max_ms": max(warm_core) if warm_core else None,
            "outer_p99_ms": percentile(outer[100:] if len(outer) > 100 else outer, .99),
            "rss_peak_mib": rss_kib / 1024.0 if rss_kib is not None else None,
        },
        "stage_timings": stage_summary,
        "artifact_hashes": hashes,
    }
    return result


def find_runs(root: Path):
    for state in sorted(root.rglob("states.csv")):
        if (state.parent / "ground_truth.csv").exists():
            yield state.parent


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    results = []
    for run in find_runs(args.input):
        result = analyze_run(run)
        (run / "validation.json").write_text(
            json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8")
        results.append(result)
    aggregate = args.input / "validation_summary.json"
    aggregate.write_text(json.dumps({"schema": "uwb-imu-pl/fde-validation-summary/v1",
                                     "runs": results}, indent=2,
                                    sort_keys=True, allow_nan=False) + "\n",
                         encoding="utf-8")
    if args.report:
        lines = ["# 五模式自动验证摘要", "",
                 "| profile | scenario | epochs | commit | pos RMSE m | finite PL | core p99 ms |",
                 "|---|---|---:|---:|---:|---:|---:|"]
        for item in results:
            c, a, i, p = item["counts"], item["accuracy"], item["integrity"], item["performance"]
            def fmt(value): return "N/A" if value is None else f"{value:.6g}"
            lines.append(f"| {item.get('profile')} | {item.get('scenario')} | {c['attempted']} | "
                         f"{c['committed']} | {fmt(a['truth_at_state_position_rmse_m'])} | "
                         f"{fmt(i['finite_pl_fraction'])} | {fmt(p['core_p99_ms'])} |")
        args.report.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"analyzed {len(results)} runs; summary={aggregate}")
    return 0 if results else 2


if __name__ == "__main__":
    raise SystemExit(main())
