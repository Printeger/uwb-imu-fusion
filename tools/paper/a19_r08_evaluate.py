#!/usr/bin/env python3
"""Independent post-freeze evaluator for one frozen R08 validation input."""

import argparse
import bisect
import csv
import datetime
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np


POLICIES = ("suppress_all", "structured_debias", "fit_only", "s_fit",
            "full_gate")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def obj(path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream, parse_constant=lambda value: (_ for _ in ()).throw(
            ValueError("non-standard JSON constant: " + value)))


def metric(value, denominator, unit):
    if value is None:
        return {"status": "UNDEFINED", "value": None, "denominator": denominator,
                "unit": unit}
    if not math.isfinite(value):
        raise ValueError("non-finite metric")
    return {"status": "AVAILABLE", "value": value,
            "denominator": denominator, "unit": unit}


def unavailable(reason, unit=""):
    return {"status": "UNAVAILABLE", "value": None, "denominator": None,
            "unit": unit, "reason": reason}


def unavailable_trajectory(reason):
    return {"rmse_m": unavailable(reason, "m"),
            "p95_m": unavailable(reason, "m"), "matches": 0,
            "association": "NOT_RUN_NO_VALID_FINAL_GRAPH"}


def rate(numerator, denominator):
    if denominator == 0:
        return {"status": "UNDEFINED", "value": None, "numerator": numerator,
                "denominator": denominator, "unit": "observation"}
    return {"status": "AVAILABLE", "value": numerator / denominator,
            "numerator": numerator, "denominator": denominator,
            "unit": "observation"}


def write_result(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w") as stream:
        stream.write(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n")
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    directory_fd = os.open(str(path.parent), os.O_DIRECTORY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def same_gate_decisions(gates):
    # Reason strings describe policy-specific tests; only group Use/Suppress
    # vectors determine whether policies made different decisions.
    return len({tuple((row[0], row[1]) for row in decisions)
                for decisions in gates.values()}) == 1


def rmse(errors):
    return math.sqrt(sum(value * value for value in errors) / len(errors))


def trajectory(path, motion, interval=None):
    estimates = []
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        values = [float(item) for item in line.split()]
        if len(values) != 8 or not all(math.isfinite(item) for item in values):
            raise ValueError("invalid trajectory row")
        if interval and not (interval[0] <= values[0] <= interval[1]):
            continue
        estimates.append((values[0], np.asarray(values[1:4], dtype=float)))
    times = [item[0] for item in motion]
    errors = []
    for time_s, estimate in estimates:
        index = bisect.bisect_left(times, time_s)
        if index < len(times) and abs(times[index] - time_s) <= 1e-12:
            truth = motion[index][1]
        elif 0 < index < len(times):
            t0, p0 = motion[index - 1]
            t1, p1 = motion[index]
            alpha = (time_s - t0) / (t1 - t0)
            truth = p0 + alpha * (p1 - p0)
        else:
            continue
        errors.append(float(np.linalg.norm(estimate - truth)))
    if not errors:
        raise ValueError("trajectory has no matched truth")
    return {"rmse_m": metric(rmse(errors), len(errors), "m"),
            "p95_m": metric(float(np.percentile(errors, 95)), len(errors), "m"),
            "matches": len(errors), "association": "EXACT_OR_LINEAR_INTERPOLATION"}


def same_gate_decisions(gates):
    # Policy-specific reason strings do not constitute a different Use/Suppress.
    vectors = {tuple((row[0], row[1]) for row in records)
               for records in gates.values()}
    return len(vectors) == 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-root", type=Path, required=True)
    parser.add_argument("--generation-manifest", type=Path, required=True)
    parser.add_argument("--freeze-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--validation-context", type=Path, required=True)
    args = parser.parse_args()
    root = args.run_root.resolve()

    freeze = obj(args.freeze_manifest)
    if freeze.get("evaluation_label") != "LIMITED_SYNTHETIC_VALIDATION" or not freeze.get(
            "decisions_and_finals_frozen"):
        raise ValueError("evaluation freeze boundary invalid")
    for item in freeze.get("payloads", []):
        path = Path(item["path"])
        if not path.is_absolute():
            path = (args.freeze_manifest.parent / path).resolve()
        if not path.is_file() or sha(path) != item["sha256"]:
            raise ValueError("frozen payload mismatch: " + str(path))

    pipeline = obj(root / "pipeline_status.json") if (root / "pipeline_status.json").is_file() else {}
    if pipeline.get("status") not in ("SCORED", "NO_CANDIDATES"):
        result = {"schema": "T10_CLOSEOUT_FAILED_EVALUATION_V1",
                  "status": "UNAVAILABLE", "reason": pipeline.get("reason", "INCOMPLETE_PRODUCER"),
                  "pipeline": pipeline, "scores": None, "policies": {},
                  "truth_read": False, "candidate_observations": None}
        write_result(args.output, result)
        print(json.dumps({"status": "FAILURE_ACCOUNTED", "truth_read": False}))
        return
    decision_manifest = obj(root / "decisions/manifest.json")
    if decision_manifest.get("truth_read") is not False:
        raise ValueError("decision manifest does not precede truth")
    export_manifest = obj(root / "stage2/stage2_export_manifest.json")
    if not all(export_manifest.get(field) is True for field in
               ("solver_converged", "export_complete", "score_complete")):
        raise ValueError("Stage2 export/score incomplete")

    # Evaluation-only files are first opened below, after every decision/final
    # payload and identity has been checked against the frozen manifest.
    truth_opened_utc = datetime.datetime.now(datetime.timezone.utc).isoformat()
    generation = obj(args.generation_manifest)
    generation_root = args.generation_manifest.resolve().parent
    motion_path = generation_root / "evaluation/motion.csv"
    if generation["base"]["role"] != "validation":
        raise ValueError("generation role is not validation")
    context = obj(args.validation_context)
    if context.get("role") != "validation" or context.get("scenario_id") != args.scenario:
        raise ValueError("validation context/scenario mismatch")
    bias_path = generation_root / generation["scenarios"][args.scenario]["truth"]
    expected_payloads = generation["payload_sha256"]
    expected_motion = expected_payloads["evaluation/motion.csv"]
    expected_bias = expected_payloads[generation["scenarios"][args.scenario]["truth"]]
    if expected_motion.startswith("sha256:"):
        expected_motion = expected_motion[7:]
    if expected_bias.startswith("sha256:"):
        expected_bias = expected_bias[7:]
    if sha(motion_path) != expected_motion or sha(bias_path) != expected_bias:
        raise ValueError("evaluation truth payload hash mismatch")

    motion = [(float(row["time_s"]), np.array([
        float(row["px_m"]), float(row["py_m"]), float(row["pz_m"])], dtype=float))
              for row in rows(motion_path)]
    bias_truth = {row["obs_id"]: float(row["total_dynamic_latent_bias_m"])
                  for row in rows(bias_path)}

    partition = rows(root / "stage1/partition.csv")
    segment_by_obs = {}
    candidate_obs = set()
    for segment in partition:
        for obs_id in filter(None, segment["obs_ids"].split(";")):
            if obs_id in segment_by_obs:
                raise ValueError("duplicate candidate obs_id")
            segment_by_obs[obs_id] = segment["segment_id"]
            candidate_obs.add(obs_id)
    amplitudes = {row["segment_id"]: float(row["amplitude_m"])
                  for row in rows(root / "segments.csv")}
    if set(segment_by_obs.values()) != set(amplitudes):
        raise ValueError("Stage2 amplitude mapping incomplete")
    if any(obs_id not in bias_truth for obs_id in candidate_obs):
        raise ValueError("candidate truth mapping incomplete")
    decision_estimate = {obs_id: amplitudes[segment_by_obs[obs_id]]
                         for obs_id in candidate_obs}
    decision_errors = {obs_id: abs(decision_estimate[obs_id] - bias_truth[obs_id])
                       for obs_id in candidate_obs}
    candidate_bias = (metric(rmse(list(decision_errors.values())),
                             len(decision_errors), "m")
                      if decision_errors else metric(None, 0, "m"))

    score_rows = rows(root / "scores.csv")
    score_summary = [{key: row[key] for key in
                      ("group_id", "ordinals", "eligible", "status", "eta",
                       "s_m", "numerical_reason", "linearization_id")}
                     for row in score_rows]
    final_times = {row["policy"]: float(row["elapsed_seconds"])
                   for row in rows(root / "final/status.csv")}
    policies = {}
    for policy in POLICIES:
        final = root / "final" / policy
        required = ("validation_status.json", "final_inference_summary.json", "final_masks.csv", "fallback_attempt.json")
        if any(not (final / name).is_file() or (final / name).stat().st_size == 0 for name in required):
            reason = ((final / "child_failure.txt").read_text().strip()
                      if (final / "child_failure.txt").is_file() else "MISSING_OR_INCOMPLETE_FINAL_EXPORT")
            policies[policy] = {
                "status": {"valid_estimate": False, "status": reason}, "summary_status": "ESTIMATION_FAILED",
                "accepted_candidate_observations": None, "final_used_candidate_observations": None,
                "trajectory_full": unavailable_trajectory(reason),
                "trajectory_historical_3_6_s": unavailable_trajectory(reason),
                "decision_time": {name: unavailable(reason) for name in
                    ("accepted_bias_rmse_m", "bad_correction_rate", "good_correction_rejection_rate")},
                "final_time": {name: unavailable(reason) for name in
                    ("applied_bias_field_rmse_m", "accepted_bias_rmse_m", "bad_correction_rate")},
                "coverage": {name: unavailable(reason) for name in
                    ("candidate_use_coverage", "eligible_use_coverage", "overall_retained_fraction")},
                "fallback": {"attempted": None, "status": "UNKNOWN_NOT_EXPORTED"},
                "elapsed_seconds": final_times.get(policy)}
            continue
        status = obj(final / "validation_status.json")
        summary = obj(final / "final_inference_summary.json")
        valid = status.get("valid_estimate") is True
        masks = rows(final / "final_masks.csv")
        candidates = [row for row in masks if row["candidate"] in ("1", "true")]
        if {row["obs_id"] for row in candidates} != candidate_obs:
            raise ValueError(policy + " candidate mask mismatch")
        accepted = {row["obs_id"] for row in candidates
                    if row["decision_use"] in ("1", "true")}
        final_used = {row["obs_id"] for row in candidates
                      if row["final_use"] in ("1", "true")}
        final_amplitudes = ({row["segment_id"]: float(row["amplitude_m"])
                             for row in rows(final / "segment_bias.csv")}
                            if valid else {})
        if valid:
            applied = {obs_id: (final_amplitudes[segment_by_obs[obs_id]]
                                if obs_id in final_used else 0.0)
                       for obs_id in candidate_obs}
            final_errors = {obs_id: abs(applied[obs_id] - bias_truth[obs_id])
                            for obs_id in candidate_obs}
            accepted_errors = [final_errors[obs_id] for obs_id in final_used]
        else:
            final_errors = {}
            accepted_errors = []
        good = {obs_id for obs_id, error in decision_errors.items()
                if error <= 0.20}
        eligible_groups = {row["group_id"] for row in rows(
            root / "decisions" / (policy + ".csv"))
                           if row["eligible"] in ("1", "true")}
        eligible_candidates = {row["obs_id"] for row in candidates
                               if row["group_id"] in eligible_groups}
        full = (trajectory(final / "trajectory.tum", motion) if valid else
                unavailable_trajectory("ESTIMATION_FAILED"))
        historical = (trajectory(final / "trajectory.tum", motion, (3.0, 6.0))
                      if valid else unavailable_trajectory("ESTIMATION_FAILED"))
        fallback = obj(final / "fallback_attempt.json")
        policies[policy] = {
            "status": status, "summary_status": summary["status"],
            "accepted_candidate_observations": len(accepted),
            "final_used_candidate_observations": len(final_used),
            "used_segments": sorted({segment_by_obs[item] for item in accepted}),
            "suppressed_segments": sorted(set(amplitudes) -
                                           {segment_by_obs[item] for item in accepted}),
            "trajectory_full": full, "trajectory_historical_3_6_s": historical,
            "decision_time": {
                "candidate_bias_field_rmse_m": candidate_bias,
                "accepted_bias_rmse_m": metric(
                    rmse([decision_errors[item] for item in accepted]), len(accepted), "m")
                    if accepted else metric(None, 0, "m"),
                "bad_correction_rate": rate(
                    sum(decision_errors[item] > 0.20 for item in accepted),
                    len(accepted)),
                "good_correction_rejection_rate": rate(
                    sum(item not in accepted for item in good), len(good))},
            "final_time": {
                "applied_bias_field_rmse_m": (metric(
                    rmse(list(final_errors.values())), len(final_errors), "m")
                    if valid and final_errors else metric(None, 0, "m")
                    if valid else unavailable("ESTIMATION_FAILED", "m")),
                "accepted_bias_rmse_m": (metric(rmse(accepted_errors),
                                                 len(accepted_errors), "m")
                    if valid and accepted_errors else metric(None, 0, "m")
                    if valid else unavailable("ESTIMATION_FAILED", "m")),
                "bad_correction_rate": (rate(
                    sum(value > 0.20 for value in accepted_errors),
                    len(accepted_errors)) if valid else
                    unavailable("ESTIMATION_FAILED", "observation"))},
            "coverage": {
                "candidate_use_coverage": rate(len(accepted), len(candidate_obs)),
                "eligible_use_coverage": rate(len(accepted),
                                               len(eligible_candidates)),
                "overall_retained_fraction": (rate(
                    sum(row["final_use"] in ("1", "true") for row in masks),
                    len(masks)) if valid else
                    unavailable("NO_VALID_FINAL_GRAPH", "observation"))},
            "fallback": fallback,
            "elapsed_seconds": final_times[policy]}

    reference = policies["suppress_all"]["trajectory_historical_3_6_s"]
    ref_rmse = reference["rmse_m"]["value"]
    ref_p95 = reference["p95_m"]["value"]
    for value in policies.values():
        historical = value["trajectory_historical_3_6_s"]
        if ref_rmse is None or historical["rmse_m"]["value"] is None:
            value["historical_delta_vs_suppress_all"] = {
                "rmse_m": unavailable("SUPPRESS_ALL_OR_POLICY_ESTIMATION_FAILED", "m"),
                "p95_m": unavailable("SUPPRESS_ALL_OR_POLICY_ESTIMATION_FAILED", "m")}
        else:
            value["historical_delta_vs_suppress_all"] = {
                "rmse_m": metric(historical["rmse_m"]["value"] - ref_rmse,
                                 historical["matches"], "m"),
                "p95_m": metric(historical["p95_m"]["value"] - ref_p95,
                                historical["matches"], "m")}

    gate_decisions = {policy: [
        (row["group_id"], row["decision"], row["reason"])
        for row in rows(root / "decisions" / (policy + ".csv"))]
                      for policy in ("fit_only", "s_fit", "full_gate")}
    los_reference = None
    if args.scenario == "los":
        final = root / "final/all_range"
        status = (obj(final / "validation_status.json") if (final / "validation_status.json").is_file()
                  else {"valid_estimate": False, "status": "ESTIMATION_FAILED",
                        "reason": ((final / "child_failure.txt").read_text().strip()
                                   if (final / "child_failure.txt").is_file()
                                   else "MISSING_FINAL_EXPORT")})
        valid = status.get("valid_estimate") is True
        all_range_full = (trajectory(final / "trajectory.tum", motion) if valid
                          else unavailable_trajectory("ESTIMATION_FAILED"))
        all_range_historical = (trajectory(final / "trajectory.tum", motion,
                                            (3.0, 6.0)) if valid
                                else unavailable_trajectory("ESTIMATION_FAILED"))
        suppress_full = policies["suppress_all"]["trajectory_full"]
        paired_valid = valid and suppress_full["rmse_m"]["value"] is not None
        los_reference = {
            "status": status, "trajectory_full": all_range_full,
            "trajectory_historical_3_6_s": all_range_historical,
            "suppress_all_delta_vs_all_range": {
                "rmse_m": metric(suppress_full["rmse_m"]["value"] -
                                 all_range_full["rmse_m"]["value"],
                                 suppress_full["matches"], "m")
                if paired_valid else unavailable("ESTIMATION_FAILED", "m"),
                "p95_m": metric(suppress_full["p95_m"]["value"] -
                                all_range_full["p95_m"]["value"],
                                suppress_full["matches"], "m")
                if paired_valid else unavailable("ESTIMATION_FAILED", "m")},
            "tolerance": {"rmse_m": 0.05, "p95_m": 0.10},
            "comparison_status": "AVAILABLE" if paired_valid else "UNAVAILABLE",
            "degradation_unacceptable": (
                suppress_full["rmse_m"]["value"] - all_range_full["rmse_m"]["value"] > .05 or
                suppress_full["p95_m"]["value"] - all_range_full["p95_m"]["value"] > .10)
                if paired_valid else None}
    result = {
        "schema": "T10_A19_R08_LIMITED_SYNTHETIC_VALIDATION_EVALUATION_V1",
        "evaluation_label": "LIMITED_SYNTHETIC_VALIDATION",
        "scenario_id": args.scenario,
        "base_trajectory_id": context["base_trajectory_id"],
        "reservation_key": context["reservation_key"],
        "truth_first_opened_after_freeze_utc": truth_opened_utc,
        "generation_manifest_sha256": sha(args.generation_manifest),
        "motion_truth_sha256": sha(motion_path), "bias_truth_sha256": sha(bias_path),
        "frame": "RAW_SYNTHETIC_GENERATOR_FRAME",
        "time_association": "EXACT_OR_LINEAR_INTERPOLATION",
        "historical_interval_closed_s": [3.0, 6.0],
        "epsilon_bad_m": 0.20, "candidate_observations": len(candidate_obs),
        "recoverability": ({"status": "not_applicable", "reason": "NOT_APPLICABLE_NO_CANDIDATES",
                            "computed": False, "eta": None, "s_m": None, "gamma": None}
                           if not candidate_obs else {"status": "executed", "computed": True}),
        "scores": score_summary, "policies": policies,
        "gate_decisions": gate_decisions, "los_all_range_reference": los_reference,
        "three_gates_identical": same_gate_decisions(gate_decisions),
        "limitations": ["TWO_VALIDATION_BASE_TRAJECTORIES_ONLY",
                        "NO_GENERALIZATION_OR_STATISTICAL_SIGNIFICANCE",
                        "NO_FORMAL_GATE_LOCK_OR_CLAIM_UPGRADE"]}
    write_result(args.output, result)
    print(json.dumps({"status": "PASS", "policies": len(policies),
                      "candidate_observations": len(candidate_obs),
                      "three_gates_identical": result["three_gates_identical"]}))


if __name__ == "__main__":
    main()
