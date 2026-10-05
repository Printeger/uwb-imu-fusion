#!/usr/bin/env python3
"""Summarize P1-01 all-attempt latency/RSS/work without dropping failures."""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import re
from typing import Iterable, Optional


def percentile(values: list[float], probability: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * probability
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def distribution(values: Iterable[float]) -> dict:
    items = list(values)
    return {
        "count": len(items),
        "p50_ms": percentile(items, 0.50),
        "p95_ms": percentile(items, 0.95),
        "p99_ms": percentile(items, 0.99),
        "max_ms": max(items) if items else None,
    }


def parse_work(path: pathlib.Path) -> dict[str, int]:
    text = path.read_text(encoding="utf-8")
    lines = [line for line in text.splitlines()
             if line.startswith("numerical_work ")]
    if len(lines) != 1:
        raise ValueError(f"{path}: expected one numerical_work line")
    result = {key: int(value) for key, value in
              re.findall(r"([a-z0-9_]+)=([0-9]+)", lines[0])}
    history_lines = [line for line in text.splitlines()
                     if line.startswith("history_root_work ")]
    if len(history_lines) > 1:
        raise ValueError(f"{path}: expected at most one history_root_work line")
    if history_lines:
        result.update({"history_root_" + key: int(value) for key, value in
                       re.findall(r"([a-z0-9_]+)=([0-9]+)", history_lines[0])})
    return result


def parse_rss(path: pathlib.Path) -> int:
    match = re.search(r"Maximum resident set size \(kbytes\): ([0-9]+)",
                      path.read_text(encoding="utf-8"))
    if not match:
        raise ValueError(f"{path}: maximum RSS is absent")
    return int(match.group(1))


def summarize_run(run_dir: pathlib.Path, stdout: pathlib.Path,
                  time_file: pathlib.Path) -> dict:
    timings: dict[int, dict[str, dict[str, object]]] = {}
    with (run_dir / "timing.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            attempt = int(row["epoch"])
            timings.setdefault(attempt, {})[row["stage"]] = {
                "wall_ms": float(row["wall_ms"]),
                "success": row["success"] == "1",
            }
    integrity: list[dict[str, str]] = []
    with (run_dir / "integrity.csv").open(newline="", encoding="utf-8") as stream:
        integrity = list(csv.DictReader(stream))
    with (run_dir / "diagnostic_attempts.csv").open(
            newline="", encoding="utf-8") as stream:
        diagnostics = list(csv.DictReader(stream))
    attempted = len(integrity)
    if len(diagnostics) != attempted:
        raise ValueError(f"{run_dir}: diagnostic attempt denominator mismatch")
    expected = list(range(1, attempted + 1))
    required_stages = ("core_total", "analysis_completion",
                       "arrival_to_publish", "outer_epoch")
    complete = [attempt for attempt in expected
                if all(stage in timings.get(attempt, {})
                       for stage in required_stages) and
                all(stage["success"]
                    for stage in timings.get(attempt, {}).values())]
    if len(timings) != attempted or len(complete) != attempted:
        raise ValueError(f"{run_dir}: incomplete all-attempt timing denominator")
    core = [timings[index]["core_total"]["wall_ms"] for index in expected]
    analysis = [timings[index]["analysis_completion"]["wall_ms"]
                for index in expected]
    publish = [timings[index]["arrival_to_publish"]["wall_ms"]
               for index in expected]
    deadline_misses = sum(int(row["deadline_missed"]) for row in integrity)
    sensor_stale = sum(int(row["watchdog_sensor_stale"])
                       for row in diagnostics)
    watchdog_refusals = sum(row["status"] == "WATCHDOG_REFUSED"
                            for row in diagnostics)
    unopened = sum(row["transaction_opened"] != "1" for row in diagnostics)
    outcomes = []
    for index, (integrity_row, diagnostic_row) in enumerate(
            zip(integrity, diagnostics), 1):
        outcomes.append({
            "attempt": index,
            "complete_work": index in complete,
            "deadline_missed": integrity_row["deadline_missed"] == "1",
            "timeout": diagnostic_row["watchdog_wall_timeout"] == "1",
            "sensor_stale": diagnostic_row["watchdog_sensor_stale"] == "1",
            "watchdog_reason": diagnostic_row["watchdog_reason"],
            "status": diagnostic_row["status"],
            "terminal_reason": diagnostic_row["reason"],
            "transaction_opened": diagnostic_row["transaction_opened"] == "1",
        })
    return {
        "run_directory": str(run_dir),
        "attempted": attempted,
        "excluded": 0,
        "complete_work_attempts": len(complete),
        "complete_work_rate": len(complete) / attempted if attempted else 0.0,
        "deadline_misses": deadline_misses,
        "deadline_miss_rate": deadline_misses / attempted if attempted else 0.0,
        "sensor_stale_attempts": sensor_stale,
        "watchdog_refused_attempts": watchdog_refusals,
        "transaction_unopened_attempts": unopened,
        "core_compute": distribution(core),
        "analysis_completion": distribution(analysis),
        "arrival_to_publish": distribution(publish),
        "rss_peak_kib": parse_rss(time_file),
        "work_counters": parse_work(stdout),
        "raw_core_ms": core,
        "raw_analysis_completion_ms": analysis,
        "raw_arrival_to_publish_ms": publish,
        "raw_attempt_outcomes": outcomes,
    }


def summarize_group(root: pathlib.Path) -> dict:
    run_dirs = sorted(path for path in root.glob("run*") if path.is_dir())
    if not run_dirs:
        run_dirs = sorted(path.parent for path in root.glob(
            "*/validation_run.json"))
    if not run_dirs:
        raise ValueError(f"{root}: no run directories")
    runs = []
    for index, run_dir in enumerate(run_dirs, 1):
        stdout = run_dir / "stdout.log"
        time_file = run_dir / "stderr.log"
        if not stdout.exists() or not time_file.exists():
            suffix = run_dir.name[3:] if run_dir.name.startswith("run") else str(index)
            stdout, time_file = (root / f"run{suffix}.stdout",
                                 root / f"run{suffix}.time")
        runs.append(summarize_run(run_dir, stdout, time_file))
    all_core = [value for run in runs for value in run["raw_core_ms"]]
    all_analysis = [value for run in runs
                    for value in run["raw_analysis_completion_ms"]]
    all_publish = [value for run in runs
                   for value in run["raw_arrival_to_publish_ms"]]
    aggregate = {
        "attempted": sum(run["attempted"] for run in runs),
        "excluded": 0,
        "complete_work_attempts": sum(run["complete_work_attempts"]
                                      for run in runs),
        "deadline_misses": sum(run["deadline_misses"] for run in runs),
        "sensor_stale_attempts": sum(run["sensor_stale_attempts"]
                                      for run in runs),
        "watchdog_refused_attempts": sum(run["watchdog_refused_attempts"]
                                          for run in runs),
        "transaction_unopened_attempts": sum(
            run["transaction_unopened_attempts"] for run in runs),
        "core_compute": distribution(all_core),
        "analysis_completion": distribution(all_analysis),
        "arrival_to_publish": distribution(all_publish),
        "rss_peak_kib": max(run["rss_peak_kib"] for run in runs),
    }
    aggregate["complete_work_rate"] = (
        aggregate["complete_work_attempts"] / aggregate["attempted"])
    aggregate["deadline_miss_rate"] = (
        aggregate["deadline_misses"] / aggregate["attempted"])
    return {"root": str(root), "runs": runs, "aggregate": aggregate}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("roots", nargs="+", type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    args = parser.parse_args()
    payload = {
        "schema": "uwb-imu-pl/p1-01-performance/v1",
        "timing_boundaries": {
            "core_compute": "production processUwbBatch wall interval",
            "analysis_completion": "same processUwbBatch return sample",
            "arrival_to_publish": "arrival through durable final logger sink",
        },
        "groups": [summarize_group(root) for root in args.roots],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
