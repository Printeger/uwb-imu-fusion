#!/usr/bin/env python3
"""Summarize development evidence without upgrading it to a formal Gate D PASS."""
import argparse
import collections
import csv
import hashlib
import json
import pathlib
import re
from gate_d_diagnostics import read_rows
from run_round2_performance import summarize_run, distribution


def compare_replays(left, right):
    def rows(path):
        values = read_rows(path.parent, path.name)
        result = {(r["repeat"], r["action_id"]): r for r in values}
        if len(result) != len(values): raise ValueError("duplicate replay action/repeat")
        return result
    a, b = rows(left), rows(right)
    if set(a) != set(b): return {"status": "INVALID", "reason": "replay action/repeat mismatch"}
    transitions = collections.Counter()
    changed_valid = 0
    worst_step_relative = None
    valid_comparisons = 0
    for key, row in a.items():
        other = b[key]
        transitions[row["reason"] + " -> " + other["reason"]] += 1
        changed_valid += row["valid"] != other["valid"]
        if row["valid"] == other["valid"] == "1":
            valid_comparisons += 1
            x, y = float(row["step_norm"]), float(other["step_norm"])
            worst_step_relative = max(worst_step_relative or 0.0, abs(x-y)/max(1e-15,abs(x),abs(y)))
    return {"status": "COMPARED", "count": len(a), "validity_changes": changed_valid,
            "reason_transitions": dict(transitions), "valid_step_norm_comparisons": valid_comparisons,
            "valid_step_norm_max_relative_difference": worst_step_relative}


def summarize(root):
    result = {"schema_version": "uwb-imu-pl/gate-d-p0-p2-development/v1",
              "status": "DEVELOPMENT_ONLY", "formal_gate_d_pass": False,
              "phase_status_semantics": "data completeness only; latency_status is the 40 ms verdict",
              "contract": {"inputs": 12000, "warmup": 100, "measured": 11900,
                           "workers": 4, "candidates_per_input": 128, "repeats": 3, "each_p99_limit_ms": 40},
              "smoke_affinity": "0,1,2,3", "replay_affinity": "4,5,6,7",
              "numeric_threads": 1, "phases": {}, "replays": {}, "comparisons": {}, "artifacts": []}
    for phase in ("p0", "p1", "p2"):
        directory = root / phase
        result["phases"][phase] = summarize_run(directory / "workers4-repeat1", 140, 4, 128)
        source = directory / "performance_summary.json"
        if source.exists():
            raw = json.loads(source.read_text())
            result["phases"][phase]["peak_child_rss_kb"] = raw.get("peak_child_rss_kb")
            result["phases"][phase]["latency_status"] = raw["status"]
        attempts_path = directory / "workers4-repeat1/diagnostic_attempts.csv"
        if attempts_path.exists():
            attempts = read_rows(attempts_path.parent, attempts_path.name)
            result["phases"][phase]["last_attempt"] = attempts[-1] if attempts else None
    for path in sorted(root.glob("p[012]_replay*_w[14].log")):
        values = [float(x) for x in re.findall(r"wall_ms=([0-9.eE+-]+)", path.read_text())]
        result["replays"][path.stem] = distribution(values)
    for attempt in (101,139,140):
        for left, right in (("p0", "p1"), ("p1", "p2")):
            a, b = (root/f"{left}_replay{attempt}_w4.csv", root/f"{right}_replay{attempt}_w4.csv")
            if a.exists() and b.exists(): result["comparisons"][f"{left}_{right}_{attempt}"] = compare_replays(a,b)
        for phase in ("p0", "p1", "p2"):
            a, b = (root/f"{phase}_replay{attempt}_w1.csv", root/f"{phase}_replay{attempt}_w4.csv")
            if a.exists() and b.exists(): result["comparisons"][f"{phase}_workers_{attempt}"] = compare_replays(a,b)
    result["transaction_comparisons"] = {}
    for left,right in (("p0","p1"),("p1","p2")):
        a_path=root/left/"workers4-repeat1/transactions.csv"
        b_path=root/right/"workers4-repeat1/transactions.csv"
        if not a_path.exists() or not b_path.exists(): continue
        a=read_rows(a_path.parent,a_path.name); b=read_rows(b_path.parent,b_path.name)
        fields=("selected_action_id","fde_status","backend_updates","stale_state","timestamp_ns")
        result["transaction_comparisons"][f"{left}_{right}"] = {
            "count_left":len(a),"count_right":len(b),
            "changed_fields":{key:sum(x[key]!=y[key] for x,y in zip(a,b)) for key in fields}}
    result["success_path_probes"] = {}
    for path in sorted((root/"success_path_probes").glob("*.stdout")):
        try: result["success_path_probes"][path.stem] = json.loads(path.read_text())
        except ValueError: result["success_path_probes"][path.stem] = {"status":"INVALID","reason":"no JSON output"}
    fixed = root/"fixed_lag_summary.json"
    if fixed.exists(): result["fixed_lag"] = json.loads(fixed.read_text())
    result["validation"] = {}
    validation = root/"validation"
    for name in ("ctest-final.log", "v2-final.log", "gate-d-tools-final.log", "p2-schema.log", "fixed-lag-schema.log"):
        path=validation/name
        if path.exists(): result["validation"][name] = {"path":str(path), "tail":path.read_text()[-2000:]}
    result["success_path_coverage"] = {
        "pipeline_successful_exclusion_observed": any(p.get("correct_exclusion",False) for p in result["success_path_probes"].values()),
        "pipeline_bridge_observed": any(p.get("bridge_epochs_observed",0)>0 for p in result["success_path_probes"].values()),
        "pipeline_union_exclusion_observed": any(p.get("union_exclusion",False) for p in result["success_path_probes"].values())}
    for path in sorted(root.rglob("*")):
        if path.is_file() and (path.suffix in (".bin", ".json", ".log") or "binary" in str(path.parent)):
            result["artifacts"].append({"path": str(path), "bytes": path.stat().st_size,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("evidence", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summarize(args.evidence), indent=2, allow_nan=False)+"\n")

if __name__ == "__main__": main()
