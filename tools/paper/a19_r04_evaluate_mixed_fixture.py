#!/usr/bin/env python3
"""Independent consumer for the R04 real mixed-final engineering fixture."""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return "sha256:" + digest.hexdigest()


def percentile95(values):
    values = sorted(values)
    return values[min(len(values) - 1, math.ceil(0.95 * len(values)) - 1)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--final-directory", required=True)
    parser.add_argument("--trajectory-truth", required=True)
    parser.add_argument("--bias-truth", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(args.final_directory).resolve()
    required = [
        "trajectory.tum", "segment_bias.csv", "final_masks.csv",
        "final_factor_audit.csv", "decisions.csv", "scores_decision.csv",
        "final_inference_summary.json", "final_content_identity.json",
    ]
    for name in required:
        if not (root / name).is_file():
            raise ValueError("missing real final artifact: " + name)

    truth_positions = {}
    with Path(args.trajectory_truth).open(encoding="utf-8") as stream:
        for line in stream:
            fields = line.split()
            if fields:
                truth_positions[float(fields[0])] = tuple(map(float, fields[1:4]))
    trajectory = {}
    with (root / "trajectory.tum").open(encoding="utf-8") as stream:
        for line in stream:
            fields = line.split()
            if fields:
                trajectory[float(fields[0])] = tuple(map(float, fields[1:4]))
    if set(trajectory) != set(truth_positions):
        raise ValueError("trajectory/truth timestamp domain mismatch")
    position_errors = [
        math.sqrt(sum((a - b) ** 2 for a, b in zip(trajectory[t], truth_positions[t])))
        for t in sorted(trajectory)
    ]

    masks = rows(root / "final_masks.csv")
    audits = {row["obs_id"]: row for row in rows(root / "final_factor_audit.csv")}
    if len(audits) != len(masks) or len({row["obs_id"] for row in masks}) != len(masks):
        raise ValueError("final ledger observation domain is not bijective")
    for row in masks:
        audit = audits.get(row["obs_id"])
        if audit is None or audit["ok"] != "1":
            raise ValueError("missing or failed final factor audit")
        if int(audit["final_factor_count"]) != int(row["final_use"]):
            raise ValueError("final mask and factor audit disagree")
    candidate = [row for row in masks if row["candidate"] == "1"]
    accepted = [row for row in candidate if row["final_use"] == "1"]
    suppressed = [row for row in candidate if row["final_use"] == "0"]
    decisions = rows(root / "decisions.csv")
    decision_kinds = sorted(row["decision"] for row in decisions)
    if decision_kinds != ["SUPPRESS", "USE"]:
        raise ValueError("fixture did not exercise mixed Use/Suppress")
    scores = rows(root / "scores_decision.csv")
    if len(scores) != len(decisions) or any(row["valid_score_exported"] != "1" for row in scores):
        raise ValueError("real decision score table is incomplete")

    with Path(args.bias_truth).open(newline="", encoding="utf-8") as stream:
        bias_truth = {row["segment_id"]: float(row["bias_m"])
                      for row in csv.DictReader(stream)}
    final_bias = {row["segment_id"]: float(row["amplitude_m"])
                  for row in rows(root / "segment_bias.csv")}
    if any(segment not in bias_truth for segment in final_bias):
        raise ValueError("final bias has no fixture truth mapping")
    bias_errors = [abs(value - bias_truth[segment])
                   for segment, value in final_bias.items()]
    summary = json.loads((root / "final_inference_summary.json").read_text())
    if summary.get("status") not in {"OK", "FALLBACK_OK"} or not summary.get("valid_estimate"):
        raise ValueError("real final did not export a valid estimate")

    result = {
        "schema": "A19_R04_REAL_MIXED_FINAL_EVALUATION_V1",
        "status": "PASS",
        "source": "INDEPENDENT_EVALUATOR_CONSUMED_CPP_FINAL_ARTIFACTS",
        "input_sha256": {name: sha256(root / name) for name in required},
        "trajectory": {
            "matched": len(position_errors),
            "raw_frame_ATE_RMSE_m": math.sqrt(sum(x * x for x in position_errors) /
                                                len(position_errors)),
            "raw_frame_ATE_P95_m": percentile95(position_errors),
        },
        "bias": {
            "final_segment_count": len(final_bias),
            "final_bias_RMSE_m": math.sqrt(sum(x * x for x in bias_errors) /
                                            len(bias_errors)),
        },
        "ledger": {
            "observation_count": len(masks),
            "candidate_count": len(candidate),
            "accepted_candidate_count": len(accepted),
            "suppressed_candidate_count": len(suppressed),
            "decision_kinds": decision_kinds,
            "factor_audit_bijective": True,
        },
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
