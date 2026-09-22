#!/usr/bin/env python3
"""D-round section 11 trace summary (roadmap 11.1/11.2).

Reads a run directory produced by a realtime driver (timing.csv plus the
events/integrity tables when present) and reports per-stage wall-time
percentiles from the RAW per-frame trace -- never a sum of component medians.

Frame classes (each derived from recorded fields, not guessed):

  cold            timing.csv `cold_warm` == "cold" (epoch <= 100)
  warm            everything after the startup cut
  warm+marg       warm epochs that logged FIXED_LAG_MARGINALIZE in events.csv
  warm+reject     warm epochs that logged UWB_REJECT in events.csv
  warm+fde        warm epochs whose recorded `fde_status` is not
                  SUCCESS_KEEP_ALL (the FDE path acted or failed closed)

Reported per class: count, p50/p95/p99/max/mean per stage, the deadline-miss
rate of core_total against the declared 40 ms target, and the finite
protection-level ratio.  Peak process numbers (Maximum resident set size,
Elapsed time) are parsed from `time.log` when the run was wrapped in
/usr/bin/time -v.

Usage:
  perf_trace_summary.py RUN_DIR [--out summary.json] [--deadline-ms 40]
"""
import argparse
import csv
import json
import math
import os
import re
import sys


def percentile(values, p):
    ordered = sorted(values)
    index = max(0, math.ceil(p * len(ordered)) - 1)
    return ordered[min(index, len(ordered) - 1)]


def stats(values):
    if not values:
        return None
    ordered = sorted(values)
    return {
        "count": len(ordered),
        "p50": percentile(ordered, 0.50),
        "p95": percentile(ordered, 0.95),
        "p99": percentile(ordered, 0.99),
        "max": ordered[-1],
        "mean": sum(ordered) / len(ordered),
    }


def load_timing(run_dir):
    rows = []
    with open(os.path.join(run_dir, "timing.csv")) as handle:
        for row in csv.DictReader(handle):
            rows.append({
                "timestamp_ns": int(row["timestamp_ns"]),
                "epoch": int(row["epoch"]),
                "stage": row["stage"],
                "wall_ms": float(row["wall_ms"]),
                "cold": row.get("cold_warm", "") == "cold",
            })
    return rows


def load_event_epochs(run_dir, wanted):
    path = os.path.join(run_dir, "events.csv")
    epochs = {}
    if not os.path.exists(path):
        return epochs
    stamps = {}
    for row in load_timing(run_dir):
        stamps.setdefault(row["timestamp_ns"], row["epoch"])
    with open(path) as handle:
        for row in csv.DictReader(handle):
            if row["event"] in wanted:
                epoch = stamps.get(int(row["timestamp_ns"]))
                if epoch is not None:
                    epochs.setdefault(row["event"], set()).add(epoch)
    return epochs


def load_fde_epochs(run_dir):
    path = os.path.join(run_dir, "integrity.csv")
    epochs = set()
    if not os.path.exists(path):
        return epochs
    stamps = {}
    for row in load_timing(run_dir):
        stamps.setdefault(row["timestamp_ns"], row["epoch"])
    with open(path) as handle:
        reader = csv.DictReader(handle)
        if "fde_status" not in (reader.fieldnames or []):
            return epochs
        for row in reader:
            if str(row["fde_status"]).strip() not in ("", "SUCCESS_KEEP_ALL"):
                epoch = stamps.get(int(row["timestamp_ns"]))
                if epoch is not None:
                    epochs.add(epoch)
    return epochs


def finite_pl_ratio(run_dir, warm_epochs):
    path = os.path.join(run_dir, "integrity.csv")
    if not os.path.exists(path):
        return None
    stamps = {}
    for row in load_timing(run_dir):
        stamps.setdefault(row["timestamp_ns"], row["epoch"])
    total = 0
    finite = 0
    with open(path) as handle:
        reader = csv.DictReader(handle)
        columns = [name for name in ("pl_x", "pl_y", "pl_z")
                   if name in (reader.fieldnames or [])]
        if not columns:
            return None
        for row in reader:
            epoch = stamps.get(int(row["timestamp_ns"]))
            if epoch is None or epoch not in warm_epochs:
                continue
            total += 1
            if all(math.isfinite(float(row[name])) for name in columns):
                finite += 1
    if total == 0:
        return None
    return {"finite": finite, "total": total, "ratio": finite / total}


def parse_time_log(run_dir):
    path = os.path.join(run_dir, "time.log")
    if not os.path.exists(path):
        return None
    with open(path) as handle:
        text = handle.read()
    peak = re.search(r"Maximum resident set size \(kbytes\): (\d+)", text)
    elapsed = re.search(r"Elapsed \(wall clock\) time .*?: ([\d:.]+)", text)
    return {
        "peak_rss_kb": int(peak.group(1)) if peak else None,
        "elapsed": elapsed.group(1) if elapsed else None,
    }


def summarize(run_dir, deadline_ms):
    timing = load_timing(run_dir)
    events = load_event_epochs(run_dir, {"FIXED_LAG_MARGINALIZE", "UWB_REJECT"})
    fde_epochs = load_fde_epochs(run_dir)
    warm_epochs = {row["epoch"] for row in timing if not row["cold"]}
    classes = {
        "cold": lambda row: row["cold"],
        "warm": lambda row: not row["cold"],
        "warm+marg": lambda row: row["epoch"] in events.get(
            "FIXED_LAG_MARGINALIZE", set()),
        "warm+reject": lambda row: row["epoch"] in events.get("UWB_REJECT",
                                                              set()),
        "warm+fde": lambda row: row["epoch"] in fde_epochs,
    }
    report = {
        "run_dir": run_dir,
        "sample_count_total": len({row["epoch"] for row in timing}),
        "deadline_ms": deadline_ms,
        "classes": {},
        "peak": parse_time_log(run_dir),
    }
    for name, selector in classes.items():
        selected = [row for row in timing if selector(row)]
        stages = {}
        for row in selected:
            stages.setdefault(row["stage"], []).append(row["wall_ms"])
        core = stats([ms for ms in
                      (row["wall_ms"] for row in selected
                       if row["stage"] == "core_total")])
        entries = {stage: stats(values) for stage, values in stages.items()}
        if core is not None:
            misses = [ms for ms in
                      (row["wall_ms"] for row in selected
                       if row["stage"] == "core_total") if ms > deadline_ms]
            core["deadline_miss_rate"] = len(misses) / core["count"]
        else:
            core = {}
        report["classes"][name] = {
            "frame_count": len({row["epoch"] for row in selected}),
            "core_total": core,
            "stages": entries,
        }
    report["finite_valid_pl_ratio"] = finite_pl_ratio(run_dir, warm_epochs)
    return report


def print_table(report):
    print(f"run: {report['run_dir']} (samples: "
          f"{report['sample_count_total']})")
    for name, data in report["classes"].items():
        core = data["core_total"]
        if not core:
            print(f"  {name:11s}: no frames")
            continue
        print(f"  {name:11s}: n={core['count']:4d} p50={core['p50']:9.2f} "
              f"p95={core['p95']:9.2f} p99={core['p99']:9.2f} "
              f"max={core['max']:9.2f} ms  miss>{report['deadline_ms']:.0f}ms="
              f"{core['deadline_miss_rate']:.3f}")
    pl = report.get("finite_valid_pl_ratio")
    if pl:
        print(f"  finite PL ratio (warm): {pl['finite']}/{pl['total']} = "
              f"{pl['ratio']:.3f}")
    peak = report.get("peak")
    if peak:
        print(f"  peak RSS: {peak['peak_rss_kb']} kB, elapsed {peak['elapsed']}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--out", default="")
    parser.add_argument("--deadline-ms", type=float, default=40.0)
    args = parser.parse_args()
    report = summarize(args.run_dir, args.deadline_ms)
    print_table(report)
    if args.out:
        with open(args.out, "w") as handle:
            json.dump(report, handle, indent=1)
        print(f"written {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
