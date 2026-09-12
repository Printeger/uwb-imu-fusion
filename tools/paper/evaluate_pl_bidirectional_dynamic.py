#!/usr/bin/env python3
"""Post-seal evaluator for bidirectional dynamic CONTROL/SHADOW admission."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import pathlib
import re
import statistics
from typing import Any


TARGET = (27956, 20276)


def load_json(path: pathlib.Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def rows(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def digest(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def state_max_delta(left: pathlib.Path, right: pathlib.Path) -> float:
    a, b = rows(left), rows(right)
    if len(a) != len(b):
        return math.inf
    identity = ("sequence", "keyframe_id", "timestamp")
    numeric = ("qx", "qy", "qz", "qw", "px", "py", "pz", "vx", "vy",
               "vz", "bax", "bay", "baz", "bgx", "bgy", "bgz")
    if any(tuple(x[k] for k in identity) != tuple(y[k] for k in identity)
           for x, y in zip(a, b)):
        return math.inf
    return max((abs(float(x[k]) - float(y[k])) for x, y in zip(a, b)
                for k in numeric), default=0.0)


def support_metrics(data: list[dict[str, str]], field: str,
                    truth_ids: set[int], start: float, end: float) -> dict[str, Any]:
    target = [x for x in data
              if (int(x["tag_id"]), int(x["anchor_id"])) == TARGET]
    selected = sorted((x for x in target if x[field] == "1"),
                      key=lambda x: (float(x["timestamp"]), int(x["obs_id"])))
    selected_ids = {int(x["obs_id"]) for x in selected}
    tp, fp = len(selected_ids & truth_ids), len(selected_ids - truth_ids)
    fn = len(truth_ids - selected_ids)
    first = float(selected[0]["timestamp"]) if selected else None
    last = float(selected[-1]["timestamp"]) if selected else None
    return {
        "TP": tp, "FP": fp, "FN": fn,
        "precision": tp / (tp + fp) if tp + fp else 0.0,
        "recall": tp / len(truth_ids),
        "start_time": first, "end_time": last,
        "onset_error_s": first - start if first is not None else None,
        "offset_error_s": last - end if last is not None else None,
        "post_injection_overshoot_s": max(0.0, last - end) if last is not None else None,
        "duration_s": last - first if first is not None and last is not None else 0.0,
    }


def ranks(values: list[float]) -> list[float]:
    indexed = sorted(enumerate(values), key=lambda pair: pair[1])
    result = [0.0] * len(values)
    i = 0
    while i < len(indexed):
        j = i + 1
        while j < len(indexed) and indexed[j][1] == indexed[i][1]:
            j += 1
        rank = (i + j - 1) / 2.0 + 1.0
        for k in range(i, j):
            result[indexed[k][0]] = rank
        i = j
    return result


def correlation(a: list[float], b: list[float]) -> float | None:
    if len(a) < 2:
        return None
    ma, mb = statistics.fmean(a), statistics.fmean(b)
    va = sum((x - ma) ** 2 for x in a)
    vb = sum((x - mb) ** 2 for x in b)
    if va == 0.0 or vb == 0.0:
        return None
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / math.sqrt(va * vb)


def forbidden_opens(root: pathlib.Path) -> tuple[int, list[str]]:
    found: list[str] = []
    tokens = ("/truth/", "ground_truth", "/oracle", "/mocap")
    for path in sorted((root / "truth_blind_access_traces").glob("*.trace")):
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if any(token in line.lower() for token in tokens) and not re.search(
                    r"= -1(?:\s|$)", line):
                found.append(f"{path.name}:{line}")
    return len(found), found


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=pathlib.Path, required=True)
    parser.add_argument("--previous-forward-root", type=pathlib.Path, required=True)
    parser.add_argument("--frozen-conditional", type=pathlib.Path, required=True)
    parser.add_argument("--truth", type=pathlib.Path, required=True)
    args = parser.parse_args()
    root, previous = args.evidence_root, args.previous_forward_root
    frozen = load_json(root / "frozen_evaluation.json")

    common = ("runtime_commit_log.csv", "measurement_decision_log.csv",
              "state_commit_log.csv", "trajectory.tum",
              "pl_conditional_innovations.csv", "pl_conditional_epochs.csv")
    comparisons: dict[str, Any] = {}
    all_exact = True
    max_state = 0.0
    for dataset in ("dynamic_clean", "dynamic_injected"):
        control, shadow = root / dataset / "control", root / dataset / "shadow"
        exact = {name: digest(control / name) == digest(shadow / name)
                 for name in common}
        delta = state_max_delta(control / "state_commit_log.csv",
                                shadow / "state_commit_log.csv")
        comparisons[dataset] = {"artifact_exact": exact,
                                "max_absolute_state_difference": delta}
        all_exact = all_exact and all(exact.values())
        max_state = max(max_state, delta)
    noninterference = {
        "schema": "pl_bidirectional_shadow_noninterference_v1",
        "datasets": comparisons,
        "measurement_decisions_identical": all(
            x["artifact_exact"]["measurement_decision_log.csv"]
            for x in comparisons.values()),
        "commit_sequences_identical": all(
            x["artifact_exact"]["runtime_commit_log.csv"]
            for x in comparisons.values()),
        "state_evolution_identical": all(
            x["artifact_exact"]["state_commit_log.csv"]
            for x in comparisons.values()),
        "scientific_outputs_exact": all_exact,
        "max_absolute_state_difference": max_state,
        "pass": all_exact and max_state <= 1e-12,
    }
    (root / "shadow_noninterference_report.json").write_text(
        json.dumps(noninterference, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    clean = load_json(root / "dynamic_clean/shadow/dynamic_status.json")
    injected = load_json(root / "dynamic_injected/shadow/dynamic_status.json")
    truth = load_json(args.truth)
    truth_ids = {int(x) for x in truth["planned_affected_obs_ids"]}
    start = float(truth["specification"]["start_s"])
    end = float(truth["specification"]["end_s"])
    candidates = rows(root / "dynamic_injected/shadow/dynamic_cusum_candidates.csv")
    final = support_metrics(candidates, "final_candidate", truth_ids, start, end)
    forward = support_metrics(candidates, "forward_candidate", truth_ids, start, end)
    backward = support_metrics(candidates, "backward_candidate", truth_ids, start, end)
    alarm_rows = [x for x in rows(root / "dynamic_injected/shadow/dynamic_cusum_trace.csv")
                  if x["threshold_crossing"] == "1" and
                  (int(x["tag_id"]), int(x["anchor_id"])) == TARGET]
    first_alarm = float(alarm_rows[0]["timestamp"]) if alarm_rows else None
    target_truth_rows = sorted((x for x in candidates if int(x["obs_id"]) in truth_ids),
                               key=lambda x: float(x["timestamp"]))
    alarm_index = next((i + 1 for i, x in enumerate(target_truth_rows)
                        if int(x["obs_id"]) == int(alarm_rows[0]["obs_id"])), None) \
        if alarm_rows else None

    frozen_signal = rows(args.frozen_conditional)
    dynamic_signal = rows(root / "dynamic_injected/shadow/dynamic_conditional_trace.csv")
    key = lambda x: (int(x["tag_id"]), int(x["anchor_id"]), int(x["obs_id"]))
    fmap, dmap = {key(x): x for x in frozen_signal}, {key(x): x for x in dynamic_signal}
    paired_keys = sorted(fmap.keys() & dmap.keys())
    fz = [float(fmap[k]["conditional_z"]) for k in paired_keys]
    dz = [float(dmap[k]["conditional_z"]) for k in paired_keys]
    deltas = [d - f for f, d in zip(fz, dz)]
    affected_keys = [k for k in paired_keys if k[2] in truth_ids]
    frozen_affected = [float(fmap[k]["conditional_z"]) for k in affected_keys]
    dynamic_affected = [float(dmap[k]["conditional_z"]) for k in affected_keys]
    signal = {
        "paired_conditional_rows": len(paired_keys),
        "missing_frozen_rows": len(dmap.keys() - fmap.keys()),
        "missing_dynamic_rows": len(fmap.keys() - dmap.keys()),
        "target_affected_paired_rows": len(affected_keys),
        "conditional_z": {
            "mean_delta": statistics.fmean(deltas) if deltas else None,
            "median_delta": statistics.median(deltas) if deltas else None,
            "max_abs_delta": max(map(abs, deltas), default=None),
            "sign_agreement": sum((x >= 0) == (y >= 0) for x, y in zip(fz, dz)) / len(fz),
            "pearson": correlation(fz, dz),
            "spearman": correlation(ranks(fz), ranks(dz)),
        },
        "target_affected": {
            "frozen_positive_fraction": sum(x > 0 for x in frozen_affected) / len(frozen_affected),
            "dynamic_positive_fraction": sum(x > 0 for x in dynamic_affected) / len(dynamic_affected),
            "frozen_cumulative_Z": sum(frozen_affected) / math.sqrt(len(frozen_affected)),
            "dynamic_cumulative_Z": sum(dynamic_affected) / math.sqrt(len(dynamic_affected)),
        },
        "forward": {"frozen": frozen["forward"], "dynamic": forward,
                    "frozen_first_alarm_index": frozen["forward_first_alarm_index"],
                    "dynamic_first_alarm_index": alarm_index},
        "backward": {"frozen": frozen["backward"], "dynamic": backward},
        "final": {"frozen": frozen["final"], "dynamic": final},
    }
    (root / "frozen_dynamic_bidirectional_comparison.json").write_text(
        json.dumps(signal, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    support = load_json(root / "dynamic_injected/shadow/dynamic_cusum_support.json")
    healthy = [x for x in support["segments"]
               if (int(x["tag_id"]), int(x["anchor_id"])) != TARGET]
    forbidden_count, forbidden_matches = forbidden_opens(root)
    dynamic_gates = {
        "D0_noninterference": noninterference["pass"],
        "D1_clean": clean["forward_alarm_count"] == 0
                    and clean["backward_alarm_count"] == 0
                    and clean["final_segment_count"] == 0,
        "D2_detection": first_alarm is not None and start <= first_alarm <= end
                        and alarm_index is not None and alarm_index <= 15,
        "D3_specificity": len(healthy) == 0,
        "D4_support": final["precision"] >= 0.80 and final["recall"] >= 0.80,
    }
    labels = {
        "D0_noninterference": "BIDIRECTIONAL_SUPPORT_FAIL_DYNAMIC_NONINTERFERENCE",
        "D1_clean": "BIDIRECTIONAL_SUPPORT_FAIL_DYNAMIC_CLEAN",
        "D2_detection": "BIDIRECTIONAL_SUPPORT_FAIL_DYNAMIC_DETECTION",
        "D3_specificity": "BIDIRECTIONAL_SUPPORT_FAIL_DYNAMIC_SUPPORT",
        "D4_support": "BIDIRECTIONAL_SUPPORT_FAIL_DYNAMIC_SUPPORT",
    }
    failures = [k for k, passed in dynamic_gates.items() if not passed]
    frozen_pass = frozen["verdict"] == "FROZEN_BIDIRECTIONAL_GATES_PASS"
    verdict = ("BIDIRECTIONAL_CUSUM_SUPPORT_PASS_FOR_PRODUCTION_ADMISSION"
               if frozen_pass and not failures and forbidden_count == 0
               else (labels[failures[0]] if failures else
                     "BIDIRECTIONAL_SUPPORT_FAIL_INTEGRITY"))
    evaluation = {
        "schema": "pl_bidirectional_cusum_final_evaluation_v1",
        "verdict": verdict,
        "frozen_gates": frozen["gates"],
        "dynamic_gates": dynamic_gates,
        "noninterference": noninterference,
        "dynamic_clean": clean,
        "dynamic_forward": forward,
        "dynamic_backward": backward,
        "dynamic_final": final,
        "dynamic_first_alarm_time": first_alarm,
        "dynamic_first_alarm_index": alarm_index,
        "dynamic_first_alarm_latency_s": first_alarm - start if first_alarm else None,
        "dynamic_healthy_final_segments": len(healthy),
        "truth_blind_audit": {"successful_forbidden_opens": forbidden_count,
                              "matches": forbidden_matches},
        "production": "NOT_MODIFIED",
        "stage2_recovery": "NOT_RUN",
        "localization": "NOT_EVALUATED",
    }
    (root / "final_evaluation.json").write_text(
        json.dumps(evaluation, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(verdict)
    return 0 if verdict.endswith("PASS_FOR_PRODUCTION_ADMISSION") else 3


if __name__ == "__main__":
    raise SystemExit(main())
