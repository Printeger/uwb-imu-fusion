#!/usr/bin/env python3
"""Post-seal truth evaluator for frozen bidirectional CUSUM support."""

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


TARGET = (27956, 20276)
FORWARD_KAPPA = 0.5
FORWARD_H = 7.0234689587858723
FORWARD_MAX_G = 17.77901339290359
FORWARD_IDENTITY = "plcusum-sha256:9b5ebeb1a7c11c1921adaafeac6d6197e7a5a142180967a4cd21fe1c461e85c3"
INTERSECTION_RULE = "EXACT_SAME_LINK_OBS_ID_FORWARD_AND_BACKWARD_V1"


def sha256(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: pathlib.Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_csv(path: pathlib.Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def verify_manifest(path: pathlib.Path) -> tuple[bool, int, list[str]]:
    checked = 0
    failures: list[str] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        expected, name = line.split(maxsplit=1)
        artifact = pathlib.Path(name)
        checked += 1
        if not artifact.is_file() or sha256(artifact) != expected:
            failures.append(str(artifact))
    return not failures, checked, failures


def verify_sibling(path: pathlib.Path) -> bool:
    seal = pathlib.Path(str(path) + ".sha256")
    return seal.is_file() and seal.read_text(encoding="utf-8").split()[0] == sha256(path)


def forbidden_opens(root: pathlib.Path) -> tuple[int, list[str]]:
    matches: list[str] = []
    tokens = ("/truth/", "ground_truth", "/oracle", "/mocap")
    for path in sorted((root / "truth_blind_access_traces").glob("*.trace")):
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            lowered = line.lower()
            if any(token in lowered for token in tokens) and not re.search(
                    r"= -1(?:\s|$)", line):
                matches.append(f"{path.name}:{line}")
    return len(matches), matches


def metrics(rows: list[dict[str, str]], candidate_field: str,
            truth_ids: set[int], start: float, end: float) -> dict[str, Any]:
    target = [row for row in rows
              if (int(row["tag_id"]), int(row["anchor_id"])) == TARGET]
    selected = [row for row in target if row[candidate_field] == "1"]
    selected.sort(key=lambda row: (float(row["timestamp"]), int(row["obs_id"])))
    selected_ids = {int(row["obs_id"]) for row in selected}
    tp = len(selected_ids & truth_ids)
    fp = len(selected_ids - truth_ids)
    fn = len(truth_ids - selected_ids)
    first = float(selected[0]["timestamp"]) if selected else None
    last = float(selected[-1]["timestamp"]) if selected else None
    return {
        "TP": tp,
        "FP": fp,
        "FN": fn,
        "precision": tp / (tp + fp) if tp + fp else 0.0,
        "recall": tp / len(truth_ids) if truth_ids else 0.0,
        "start_time": first,
        "end_time": last,
        "onset_error_s": first - start if first is not None else None,
        "offset_error_s": last - end if last is not None else None,
        "pre_injection_leakage_s": max(0.0, start - first) if first is not None else None,
        "post_injection_overshoot_s": max(0.0, last - end) if last is not None else None,
        "candidate_duration_s": last - first if first is not None and last is not None else 0.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence-root", type=pathlib.Path, required=True)
    parser.add_argument("--previous-forward-root", type=pathlib.Path, required=True)
    parser.add_argument("--truth", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--previous-verification-output", type=pathlib.Path, required=True)
    args = parser.parse_args()

    root = args.evidence_root
    previous = args.previous_forward_root
    old_seal_ok, old_count, old_failures = verify_manifest(
        previous / "sealed_detector_hashes.sha256")
    old_calibration = read_json(previous / "cusum_calibration.json")
    old_status = read_json(previous / "frozen_injected/cusum_status.json")
    replay_status = read_json(root / "forward_regression_replay/cusum_status.json")
    forward_byte_equal = all(
        (previous / "frozen_injected" / name).read_bytes()
        == (root / "forward_regression_replay" / name).read_bytes()
        for name in ("cusum_trace.csv", "cusum_candidates.csv",
                     "cusum_support.json", "support_partition.json",
                     "cusum_status.json"))
    previous_verification = {
        "schema": "pl_forward_evidence_verification_v1",
        "seal_sha256": sha256(previous / "sealed_detector_hashes.sha256"),
        "sealed_artifacts_verified": old_count,
        "seal_valid": old_seal_ok,
        "failures": old_failures,
        "forward_replay_five_artifacts_byte_identical": forward_byte_equal,
        "forward_detector_identity": old_status["detector_identity"],
        "forward_kappa": old_status["kappa"],
        "forward_h": old_status["h_locked"],
    }
    args.previous_verification_output.write_text(
        json.dumps(previous_verification, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    backward_calibration = read_json(root / "backward_cusum_calibration.json")
    clean_status = read_json(root / "backward_clean_validation/backward_status.json")
    backward_status = read_json(root / "backward_injected/backward_status.json")
    final_status = read_json(root / "frozen_bidirectional_support/bidirectional_status.json")
    backward_trace = read_csv(root / "backward_injected/backward_cusum_trace.csv")
    final_rows = read_csv(root / "frozen_bidirectional_support/bidirectional_support.csv")
    forward_trace = read_csv(previous / "frozen_injected/cusum_trace.csv")

    # Truth becomes visible only here, after backward and final support seals.
    truth = read_json(args.truth)
    truth_ids = {int(value) for value in truth["planned_affected_obs_ids"]}
    specification = truth["specification"]
    start = float(specification["start_s"])
    end = float(specification["end_s"])
    forward_metrics = metrics(forward_trace, "candidate", truth_ids, start, end)
    backward_metrics = metrics(backward_trace, "backward_candidate", truth_ids, start, end)
    final_metrics = metrics(final_rows, "final_candidate", truth_ids, start, end)
    final_support = read_json(root / "frozen_bidirectional_support/bidirectional_support.json")
    healthy_final = [segment for segment in final_support["segments"]
                     if (segment["tag_id"], segment["anchor_id"]) != TARGET]
    target_final = [segment for segment in final_support["segments"]
                    if (segment["tag_id"], segment["anchor_id"]) == TARGET]
    backward_crossings = [row for row in backward_trace
                          if row["threshold_crossing"] == "1"
                          and (int(row["tag_id"]), int(row["anchor_id"])) == TARGET]
    forbidden_count, forbidden_matches = forbidden_opens(root)

    h_expected = max(5.0, float(backward_calibration["B_calibration_max"]) + 1.0)
    b0_checks = {
        "previous_forward_seal_valid": old_seal_ok,
        "forward_identity_unchanged": old_status["detector_identity"] == FORWARD_IDENTITY,
        "forward_kappa_unchanged": float(old_status["kappa"]) == FORWARD_KAPPA,
        "forward_h_unchanged": float(old_status["h_locked"]) == FORWARD_H,
        "original_split_unchanged": backward_calibration[
            "original_clean_split_manifest_hash"] == old_calibration[
                "clean_split_manifest_hash"],
        "backward_calibration_sealed": verify_sibling(root / "backward_cusum_calibration.json"),
        "backward_kappa_locked": float(backward_calibration["kappa_backward"]) == 0.5,
        "backward_threshold_rule_exact": math.isclose(
            float(backward_calibration["h_backward"]), h_expected,
            rel_tol=0.0, abs_tol=1e-15),
        "truth_forbidden_opens_zero": forbidden_count == 0,
        "final_rule_exact": final_status["final_intersection_rule"] == INTERSECTION_RULE,
        "all_30_truth_ids_available": len(truth_ids) == 30,
        "all_30_truth_ids_in_rows": len({int(row["obs_id"]) for row in final_rows} & truth_ids) == 30,
    }
    b0 = all(b0_checks.values())
    b1 = bool(
        forward_byte_equal
        and forward_metrics["TP"] == 30 and forward_metrics["FP"] == 37
        and forward_metrics["FN"] == 0
        and math.isclose(float(replay_status["max_G"]), FORWARD_MAX_G,
                         rel_tol=0.0, abs_tol=1e-14)
        and replay_status["alarm_link_count"] == 1)
    b2 = clean_status["alarm_count"] == 0 and clean_status["segment_count"] == 0
    b3 = bool(len(target_final) >= 1 and final_metrics["precision"] >= 0.80
              and final_metrics["recall"] >= 0.80)
    b4 = len(healthy_final) == 0
    gates = {"B0": b0, "B1": b1, "B2": b2, "B3": b3, "B4": b4}
    verdicts = {
        "B0": "BIDIRECTIONAL_SUPPORT_FAIL_INTEGRITY",
        "B1": "BIDIRECTIONAL_SUPPORT_FAIL_FORWARD_REGRESSION",
        "B2": "BIDIRECTIONAL_SUPPORT_FAIL_BACKWARD_CLEAN_VALIDATION",
        "B3": "BIDIRECTIONAL_SUPPORT_FAIL_TARGET_SUPPORT",
        "B4": "BIDIRECTIONAL_SUPPORT_FAIL_SPECIFICITY",
    }
    failures = [gate for gate, passed in gates.items() if not passed]
    verdict = verdicts[failures[0]] if failures else "FROZEN_BIDIRECTIONAL_GATES_PASS"
    result = {
        "schema": "pl_bidirectional_cusum_frozen_evaluation_v1",
        "verdict": verdict,
        "gates": gates,
        "B0_checks": b0_checks,
        "forward": forward_metrics,
        "forward_first_alarm_index": 15,
        "forward_first_alarm_latency_s": 3.6047780513763428,
        "forward_max_G": replay_status["max_G"],
        "backward_calibration": backward_calibration,
        "backward_clean_validation": {
            "alarms": clean_status["alarm_count"],
            "segments": clean_status["segment_count"],
            "max_B": clean_status["max_B"],
        },
        "backward": backward_metrics,
        "backward_threshold_crossing_time":
            float(backward_crossings[0]["timestamp"]) if backward_crossings else None,
        "final": final_metrics,
        "target_final_segments": len(target_final),
        "healthy_final_segments": len(healthy_final),
        "truth_blind_audit": {
            "successful_forbidden_opens": forbidden_count,
            "matches": forbidden_matches,
        },
        "dynamic_admission": "ELIGIBLE_TO_RUN" if not failures
        else "NOT_RUN_DUE_TO_EARLIER_FAILURE",
        "production": "NOT_MODIFIED",
        "stage2_recovery": "NOT_RUN",
        "localization": "NOT_EVALUATED",
    }
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    print(verdict)
    return 0 if not failures else 3


if __name__ == "__main__":
    sys.exit(main())
