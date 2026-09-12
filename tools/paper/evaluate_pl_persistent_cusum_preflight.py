#!/usr/bin/env python3
"""Independent truth-stage evaluator for the locked CUSUM preflight.

This program is intentionally separate from split/calibration/detector
processes. It may read injection truth only after detector artifacts are
sealed, and it never constructs conditional_z, CUSUM state, or support.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import pathlib
import re
import sys
from typing import Any


TARGET_TAG = 27956
TARGET_ANCHOR = 20276
KAPPA = 0.5


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: pathlib.Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def load_csv(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def verify_one_line_seal(path: pathlib.Path) -> bool:
    seal = pathlib.Path(str(path) + ".sha256")
    if not seal.is_file():
        return False
    expected = seal.read_text(encoding="utf-8").split()[0]
    return expected == sha256(path)


def verify_manifest(seal: pathlib.Path, base: pathlib.Path) -> bool:
    if not seal.is_file():
        return False
    for line in seal.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        expected, name = line.split(maxsplit=1)
        path = pathlib.Path(name)
        if not path.is_absolute():
            path = base / path
        if not path.is_file() or sha256(path) != expected:
            return False
    return True


def forbidden_successes(paths: list[pathlib.Path]) -> tuple[int, list[str]]:
    tokens = ("/truth/", "ground_truth", "/oracle", "/mocap")
    hits: list[str] = []
    for path in paths:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            lowered = line.lower()
            if any(token in lowered for token in tokens) and not re.search(
                    r"= -1(?:\s|$)", line):
                hits.append(f"{path.name}:{line}")
    return len(hits), hits


def alarm_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    return [row for row in rows if row["threshold_crossing"] == "1"]


def segment_metrics(trace: list[dict[str, str]], truth: dict[str, Any]) -> dict[str, Any]:
    affected = {int(value) for value in truth["planned_affected_obs_ids"]}
    specification = truth["specification"]
    target_rows = [
        row for row in trace
        if int(row["tag_id"]) == TARGET_TAG
        and int(row["anchor_id"]) == TARGET_ANCHOR
    ]
    target_candidates = [row for row in target_rows if row["candidate"] == "1"]
    candidate_ids = {int(row["obs_id"]) for row in target_candidates}
    tp = len(candidate_ids & affected)
    fp = len(candidate_ids - affected)
    fn = len(affected - candidate_ids)
    precision = tp / (tp + fp) if tp + fp else 0.0
    recall = tp / len(affected) if affected else 0.0

    affected_rows = [row for row in target_rows if int(row["obs_id"]) in affected]
    affected_rows.sort(key=lambda row: (
        float(row["timestamp"]), int(row["keyframe_id"]),
        int(row["source_order"]), int(row["obs_id"])))
    affected_index = {int(row["obs_id"]): i + 1 for i, row in enumerate(affected_rows)}
    target_alarms = alarm_rows(target_rows)
    first_alarm = target_alarms[0] if target_alarms else None
    start = float(specification["start_s"])
    end = float(specification["end_s"])
    first_alarm_time = float(first_alarm["timestamp"]) if first_alarm else None
    first_alarm_index = affected_index.get(int(first_alarm["obs_id"])) if first_alarm else None
    onset = float(target_candidates[0]["timestamp"]) if target_candidates else None
    candidate_end = float(target_candidates[-1]["timestamp"]) if target_candidates else None
    return {
        "affected_truth_count": len(affected),
        "affected_runtime_count": len(affected_rows),
        "all_affected_present": len(affected_rows) == len(affected),
        "first_alarm_time": first_alarm_time,
        "first_alarm_obs_id": int(first_alarm["obs_id"]) if first_alarm else None,
        "first_alarm_affected_index": first_alarm_index,
        "first_alarm_latency_s": first_alarm_time - start if first_alarm else None,
        "first_alarm_within_interval": bool(
            first_alarm and start <= first_alarm_time <= end),
        "estimated_onset_time": onset,
        "estimated_onset_error_s": onset - start if onset is not None else None,
        "candidate_end_time": candidate_end,
        "segment_end_overshoot_s": candidate_end - end if candidate_end is not None else None,
        "candidate_duration_s": candidate_end - onset
        if candidate_end is not None and onset is not None else 0.0,
        "TP": tp,
        "FP": fp,
        "FN": fn,
        "precision": precision,
        "recall": recall,
    }


def evaluate_frozen(args: argparse.Namespace) -> dict[str, Any]:
    evidence = args.evidence_root
    source = args.locked_signal_root
    split = load_json(evidence / "clean_split_manifest.json")
    calibration = load_json(evidence / "cusum_calibration.json")
    clean_status = load_json(evidence / "frozen_clean_validation/cusum_status.json")
    injected_status = load_json(evidence / "frozen_injected/cusum_status.json")
    clean_trace = load_csv(evidence / "frozen_clean_validation/cusum_trace.csv")
    injected_trace = load_csv(evidence / "frozen_injected/cusum_trace.csv")

    # Only now, after all detector artifacts were sealed, read truth.
    truth = load_json(args.truth)
    metrics = segment_metrics(injected_trace, truth)
    forbidden_count, forbidden_lines = forbidden_successes([
        evidence / "clean_split_file_access.trace",
        evidence / "clean_calibration_file_access.trace",
        evidence / "clean_validation_detector_file_access.trace",
        evidence / "injected_detector_file_access.trace",
    ])

    h_expected = max(5.0, float(calibration["G_calibration_max"]) + 1.0)
    locked_seal_ok = verify_manifest(
        source / "sealed_statistics_hashes.sha256", source)
    input_hashes_ok = (
        split["input_sha256"] == "sha256:" + sha256(
            source / "pl_conditional_innovations_clean.csv")
        and injected_status["input_hash"] == "sha256:" + sha256(
            source / "pl_conditional_innovations_injected.csv")
    )
    target_alarms = [
        row for row in alarm_rows(injected_trace)
        if int(row["tag_id"]) == TARGET_TAG and int(row["anchor_id"]) == TARGET_ANCHOR
    ]
    non_target_alarms = [
        row for row in alarm_rows(injected_trace)
        if (int(row["tag_id"]), int(row["anchor_id"]))
        != (TARGET_TAG, TARGET_ANCHOR)
    ]
    support = load_json(evidence / "frozen_injected/cusum_support.json")
    target_segments = [
        segment for segment in support["segments"]
        if (segment["tag_id"], segment["anchor_id"])
        == (TARGET_TAG, TARGET_ANCHOR)
    ]
    healthy_segments = [
        segment for segment in support["segments"]
        if (segment["tag_id"], segment["anchor_id"])
        != (TARGET_TAG, TARGET_ANCHOR)
    ]

    f0_checks = {
        "locked_input_hashes_verified": locked_seal_ok and input_hashes_ok,
        "split_manifest_sealed": verify_one_line_seal(evidence / "clean_split_manifest.json"),
        "calibration_sealed": verify_one_line_seal(evidence / "cusum_calibration.json"),
        "frozen_outputs_sealed": verify_manifest(
            evidence / "sealed_frozen_detector_hashes.sha256", pathlib.Path("/")),
        "temporal_partitions_disjoint":
            float(split["calibration_end"]) < float(split["validation_start"]),
        "guard_band_exactly_one_second": math.isclose(
            float(split["validation_start"]) - float(split["calibration_end"]),
            1.0, rel_tol=0.0, abs_tol=1e-12),
        "kappa_locked": float(calibration["kappa"]) == KAPPA,
        "threshold_rule_exact": math.isclose(
            float(calibration["h_locked"]), h_expected,
            rel_tol=0.0, abs_tol=1e-15),
        "group_chi_square_unused":
            clean_status["group_statistic_used_for_decision"] is False
            and injected_status["group_statistic_used_for_decision"] is False,
        "detector_truth_access_zero":
            clean_status["truth_access_count"] == 0
            and injected_status["truth_access_count"] == 0
            and forbidden_count == 0,
        "detector_identity_equal":
            clean_status["detector_identity"] == injected_status["detector_identity"],
        "all_30_affected_target_ids_present": metrics["all_affected_present"]
            and metrics["affected_truth_count"] == 30,
    }
    f0 = all(f0_checks.values())
    f1 = clean_status["alarm_count"] == 0 and clean_status["segment_count"] == 0
    f2 = bool(
        target_alarms and metrics["first_alarm_within_interval"]
        and metrics["first_alarm_affected_index"] is not None
        and metrics["first_alarm_affected_index"] <= 15)
    f3 = len(non_target_alarms) == 0 and len(healthy_segments) == 0
    f4 = bool(
        metrics["precision"] >= 0.80 and metrics["recall"] >= 0.80
        and len(target_segments) >= 1 and len(healthy_segments) == 0)
    gates = {"F0": f0, "F1": f1, "F2": f2, "F3": f3, "F4": f4}
    failures = [name for name, passed in gates.items() if not passed]
    failure_verdict = {
        "F0": "CUSUM_PREFLIGHT_FAIL_INTEGRITY",
        "F1": "CUSUM_PREFLIGHT_FAIL_CLEAN_VALIDATION",
        "F2": "CUSUM_PREFLIGHT_FAIL_TARGET_MISSED_OR_LATE",
        "F3": "CUSUM_PREFLIGHT_FAIL_ANCHOR_SPECIFICITY",
        "F4": "CUSUM_PREFLIGHT_FAIL_SUPPORT_QUALITY",
    }
    verdict = "FROZEN_GATES_PASS" if not failures else failure_verdict[failures[0]]
    return {
        "schema": "pl_persistent_cusum_frozen_evaluation_v1",
        "verdict": verdict,
        "gates": gates,
        "dynamic_shadow_replay": "NOT_RUN_DUE_TO_EARLIER_FAILURE"
        if failures else "ELIGIBLE_TO_RUN",
        "dynamic_gates": {
            name: "NOT_RUN_DUE_TO_EARLIER_FAILURE"
            if failures else "PENDING"
            for name in ("D0", "D1", "D2", "D3", "D4")
        },
        "F0_checks": f0_checks,
        "split": split,
        "calibration": calibration,
        "held_out_clean_validation": {
            "alarm_count": clean_status["alarm_count"],
            "segment_count": clean_status["segment_count"],
            "max_G": clean_status["max_G"],
        },
        "frozen_target": metrics,
        "frozen_target_max_G": next(
            item["max_G"] for item in injected_status["per_link_max_G"]
            if item["tag_id"] == TARGET_TAG and item["anchor_id"] == TARGET_ANCHOR),
        "frozen_healthy": {
            "alarm_links": len({(row["tag_id"], row["anchor_id"])
                                for row in non_target_alarms}),
            "segments": len(healthy_segments),
        },
        "truth_blind_audit": {
            "successful_forbidden_opens": forbidden_count,
            "matches": forbidden_lines,
        },
        "production_integration": "NOT_RUN",
        "stage2_recovery": "NOT_RUN",
        "localization_accuracy": "NOT_EVALUATED",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=("frozen",), required=True)
    parser.add_argument("--evidence-root", type=pathlib.Path, required=True)
    parser.add_argument("--locked-signal-root", type=pathlib.Path, required=True)
    parser.add_argument("--truth", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    result = evaluate_frozen(args)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(result["verdict"])
    return 0 if result["verdict"] == "FROZEN_GATES_PASS" else 3


if __name__ == "__main__":
    sys.exit(main())
