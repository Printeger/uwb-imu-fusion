#!/usr/bin/env python3
"""T09 run-unit-aware evaluator with explicit NA and denominator semantics."""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
import yaml


def load_stage2_cache_identity():
    """Load the scheduler's canonical v2 implementation from this directory."""
    scheduler_path = Path(__file__).resolve().with_name("run_experiments.py")
    spec = importlib.util.spec_from_file_location(
        "uifgo_t09_scheduler_cache_identity", scheduler_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load canonical Stage-2 cache identity")
    scheduler = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(scheduler)
    return scheduler.stage2_cache_identity


stage2_cache_identity = load_stage2_cache_identity()


def metric(value=None, status="AVAILABLE", numerator=None, denominator=None,
           unit="", reason=""):
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("non-finite metric value")
    return {"value": value, "status": status, "numerator": numerator,
            "denominator": denominator, "unit": unit, "reason": reason}


def ratio(numerator: int, denominator: int, unit="run"):
    if denominator == 0:
        return metric(None, "UNDEFINED_ZERO_DENOMINATOR", numerator,
                      denominator, unit, "ZERO_DENOMINATOR")
    return metric(numerator / denominator, "AVAILABLE", numerator,
                  denominator, unit, "")


def unavailable(status, reason, unit=""):
    return metric(None, status, None, None, unit, reason)


def atomic_json(path: Path, value):
    staging = path.with_suffix(path.suffix + ".staging")
    with staging.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    staging.replace(path)


def read_json(path: Path):
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"JSON object expected: {path}")
    return value


def file_sha256(path: Path):
    import hashlib
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return "sha256:" + digest.hexdigest()


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])


def pose_matrix(row):
    T = np.eye(4)
    T[:3, :3] = quaternion_matrix(row[2])
    T[:3, 3] = row[1]
    return T


def rotation_angle(R):
    cosine = min(1.0, max(-1.0, (float(np.trace(R)) - 1.0) / 2.0))
    return math.acos(cosine)


def se3_relative_pose_error(est_i, est_j, gt_i, gt_j):
    delta_est = np.linalg.inv(pose_matrix(est_i)) @ pose_matrix(est_j)
    delta_gt = np.linalg.inv(pose_matrix(gt_i)) @ pose_matrix(gt_j)
    error = np.linalg.inv(delta_gt) @ delta_est
    return float(np.linalg.norm(error[:3, 3])), rotation_angle(error[:3, :3])


def cell_is_comparable(cell):
    return cell.get("status") != "COMPARABILITY_INVALID"


def slerp(q0, q1, alpha):
    q0 = np.asarray(q0, dtype=float); q0 /= np.linalg.norm(q0)
    q1 = np.asarray(q1, dtype=float); q1 /= np.linalg.norm(q1)
    dot = float(q0 @ q1)
    if dot < 0:
        q1, dot = -q1, -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        q = q0 + alpha * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = math.acos(dot)
    return (math.sin((1-alpha)*theta) * q0 + math.sin(alpha*theta) * q1) / math.sin(theta)


def load_tum(path: Path):
    rows = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            values = [float(item) for item in line.split()]
            if len(values) != 8 or not all(math.isfinite(item) for item in values):
                raise ValueError(f"invalid TUM row in {path}")
            rows.append((values[0], np.array(values[1:4]), np.array(values[4:8])))
    if any(rows[i][0] >= rows[i+1][0] for i in range(len(rows)-1)):
        raise ValueError(f"trajectory timestamps are not strictly increasing: {path}")
    return rows


def transform_point(T, position, quaternion, lever):
    local = position + quaternion_matrix(quaternion) @ lever
    return T[:3, :3] @ local + T[:3, 3]


def match_trajectories(est, gt, protocol):
    policy = protocol.get("policy")
    tolerance = float(protocol.get("tolerance_s", 0.0))
    max_gap = float(protocol.get("max_bracket_gap_s", 0.0))
    matched, deltas = [], []
    gt_times = np.array([row[0] for row in gt])
    for est_row in est:
        t = est_row[0]
        index = int(np.searchsorted(gt_times, t))
        if policy == "nearest_within_tolerance":
            candidates = [i for i in (index-1, index) if 0 <= i < len(gt)]
            if not candidates:
                continue
            best = min(candidates, key=lambda i: (abs(gt[i][0]-t), i))
            delta = abs(gt[best][0] - t)
            if delta <= tolerance:
                matched.append((est_row, gt[best])); deltas.append(delta)
        elif policy == "bracketed_interpolation":
            if index == 0 or index == len(gt):
                continue
            left, right = gt[index-1], gt[index]
            gap = right[0] - left[0]
            if gap <= 0 or gap > max_gap:
                continue
            alpha = (t-left[0]) / gap
            interp = (t, left[1] + alpha*(right[1]-left[1]),
                      slerp(left[2], right[2], alpha))
            matched.append((est_row, interp)); deltas.append(min(t-left[0], right[0]-t))
        else:
            raise ValueError("unknown time association policy")
    return matched, deltas


def summarize_errors(errors, prefix):
    errors = np.asarray(errors, dtype=float)
    return {
        f"{prefix}_rmse_m": metric(float(np.sqrt(np.mean(errors**2))), unit="m"),
        f"{prefix}_mean_m": metric(float(np.mean(errors)), unit="m"),
        f"{prefix}_p50_m": metric(float(np.percentile(errors, 50)), unit="m"),
        f"{prefix}_p95_m": metric(float(np.percentile(errors, 95)), unit="m"),
        f"{prefix}_p99_m": metric(float(np.percentile(errors, 99)), unit="m"),
        f"{prefix}_max_m": metric(float(np.max(errors)), unit="m"),
    }


def trajectory_metrics(run_dir: Path, scenario: dict):
    trajectory = run_dir / "trajectory.tum"
    if not trajectory.is_file():
        reason = "VALID_FINAL_TRAJECTORY_MISSING"
        na = unavailable("UNAVAILABLE_ESTIMATION_FAILURE", reason, "m")
        return {name: dict(na) for name in ("raw_frame_ATE_rmse_m",
                                             "aligned_ATE_rmse_m", "RPE_rmse_m",
                                             "RPE_rotation_rmse_rad")}
    if not scenario or not scenario.get("ground_truth"):
        raw_na = unavailable("UNAVAILABLE_FRAME_OR_POINT_PROVENANCE",
                             "EVALUATION_MANIFEST_OR_GT_MISSING", "m")
        return {"raw_frame_ATE_rmse_m": raw_na,
                "aligned_ATE_rmse_m": unavailable(
                    "UNAVAILABLE_INSUFFICIENT_MATCHES", "GT_MISSING", "m"),
                "RPE_rmse_m": unavailable(
                    "UNAVAILABLE_INSUFFICIENT_MATCHES", "GT_MISSING", "m"),
                "RPE_rotation_rmse_rad": unavailable(
                    "UNAVAILABLE_INSUFFICIENT_MATCHES", "GT_MISSING", "rad")}
    gt_path = Path(scenario["ground_truth"])
    est, gt = load_tum(trajectory), load_tum(gt_path)
    matches, deltas = match_trajectories(est, gt, scenario["time_association"])
    result = {"matched_count": metric(len(matches), unit="pose"),
              "unmatched_count": metric(len(est)-len(matches), unit="pose"),
              "evaluation_interval_s": (metric(
                  [matches[0][0][0], matches[-1][0][0]], unit="s")
                  if matches else unavailable(
                      "UNAVAILABLE_INSUFFICIENT_MATCHES", "NO_TIME_MATCHES", "s")),
              "time_delta_mean_s": (metric(float(np.mean(deltas)), unit="s")
                                      if deltas else unavailable(
                                          "UNAVAILABLE_INSUFFICIENT_MATCHES",
                                          "NO_TIME_MATCHES", "s")),
              "time_delta_p95_s": (metric(float(np.percentile(deltas, 95)), unit="s")
                                     if deltas else unavailable(
                                         "UNAVAILABLE_INSUFFICIENT_MATCHES",
                                         "NO_TIME_MATCHES", "s")),
              "time_delta_max_s": (metric(float(np.max(deltas)), unit="s")
                                     if deltas else unavailable(
                                         "UNAVAILABLE_INSUFFICIENT_MATCHES",
                                         "NO_TIME_MATCHES", "s"))}
    if len(matches) < 3:
        na = unavailable("UNAVAILABLE_INSUFFICIENT_MATCHES",
                         "FEWER_THAN_THREE_MATCHES", "m")
        result.update({"raw_frame_ATE_rmse_m": dict(na),
                       "aligned_ATE_rmse_m": dict(na), "RPE_rmse_m": dict(na),
                       "RPE_rotation_rmse_rad": unavailable(
                           "UNAVAILABLE_INSUFFICIENT_MATCHES",
                           "FEWER_THAN_THREE_MATCHES", "rad")})
        return result
    T_GE_raw = scenario.get("T_G_E")
    T_Q_I_raw = scenario.get("T_Q_I")
    provenance = scenario.get("frame_point_provenance")
    if T_GE_raw is None or T_Q_I_raw is None or not provenance:
        result["raw_frame_ATE_rmse_m"] = unavailable(
            "UNAVAILABLE_FRAME_OR_POINT_PROVENANCE",
            "T_G_E_T_Q_I_OR_PROVENANCE_MISSING", "m")
        T_GE = np.eye(4); T_QI = np.eye(4)
    else:
        T_GE = np.asarray(T_GE_raw, dtype=float).reshape(4, 4)
        T_QI = np.asarray(T_Q_I_raw, dtype=float).reshape(4, 4)
        if not np.all(np.isfinite(T_GE)) or not np.all(np.isfinite(T_QI)):
            raise ValueError("evaluation transforms are non-finite")
    lever_qi = T_QI[:3, 3]
    est_raw = np.array([transform_point(T_GE, e[1], e[2], np.zeros(3))
                        for e, _ in matches])
    gt_points = np.array([transform_point(np.eye(4), g[1], g[2], lever_qi)
                          for _, g in matches])
    if T_GE_raw is not None and T_Q_I_raw is not None and provenance:
        raw_errors = np.linalg.norm(est_raw-gt_points, axis=1)
        result.update(summarize_errors(np.linalg.norm((est_raw-gt_points)[:,:2],axis=1), "raw_frame_horizontal"))
        result.update(summarize_errors(np.abs((est_raw-gt_points)[:,2]), "raw_frame_height"))
        result.update(summarize_errors(raw_errors, "raw_frame_ATE"))
        result["raw_frame_axis_rmse_m"] = metric(
            [float(x) for x in np.sqrt(np.mean((est_raw-gt_points)**2, axis=0))],
            unit="m_xyz")
    est_native = np.array([e[1] for e, _ in matches])
    est_centered = est_native - est_native.mean(axis=0)
    gt_centered = gt_points - gt_points.mean(axis=0)
    covariance = est_centered.T @ gt_centered
    U, _, Vt = np.linalg.svd(covariance)
    D = np.eye(3); D[2, 2] = np.linalg.det(Vt.T @ U.T)
    rotation = Vt.T @ D @ U.T
    aligned = (rotation @ est_native.T).T + gt_points.mean(axis=0) - rotation @ est_native.mean(axis=0)
    if np.linalg.matrix_rank(est_centered) < 2:
        result["aligned_ATE_rmse_m"] = unavailable(
            "UNAVAILABLE_INSUFFICIENT_MATCHES", "SE3_ALIGNMENT_DEGENERATE", "m")
    else:
        result.update(summarize_errors(np.linalg.norm((aligned-gt_points)[:,:2],axis=1), "aligned_horizontal"))
        result.update(summarize_errors(np.abs((aligned-gt_points)[:,2]), "aligned_height"))
        result.update(summarize_errors(np.linalg.norm(aligned-gt_points, axis=1),
                                       "aligned_ATE"))
    horizon = float(scenario.get("rpe_horizon_s", 1.0))
    rpe_translation = []
    rpe_rotation = []
    times = [pair[0][0] for pair in matches]
    for i, t in enumerate(times):
        target = t + horizon
        j = int(np.searchsorted(times, target))
        if j < len(times) and abs(times[j]-target) <= float(
                scenario["time_association"].get("tolerance_s", 0.0)):
            est_i = matches[i][0]
            est_j = matches[j][0]
            gt_i = matches[i][1]
            gt_j = matches[j][1]
            # Express both inputs as the declared common rigid body/frame.
            est_i_T = T_GE @ pose_matrix(est_i)
            est_j_T = T_GE @ pose_matrix(est_j)
            gt_i_T = pose_matrix(gt_i) @ T_QI
            gt_j_T = pose_matrix(gt_j) @ T_QI
            canonical = lambda t, T: (t, T[:3, 3], matrix_quaternion(T[:3, :3]))
            translation, rotation_error = se3_relative_pose_error(
                canonical(est_i[0], est_i_T), canonical(est_j[0], est_j_T),
                canonical(gt_i[0], gt_i_T), canonical(gt_j[0], gt_j_T))
            rpe_translation.append(translation)
            rpe_rotation.append(rotation_error)
    result["RPE_rmse_m"] = (metric(float(np.sqrt(np.mean(np.square(rpe_translation)))), unit="m")
                            if rpe_translation else unavailable(
                                "UNAVAILABLE_INSUFFICIENT_MATCHES",
                                "NO_VALID_RPE_HORIZON_PAIR", "m"))
    result["RPE_rotation_rmse_rad"] = (
        metric(float(np.sqrt(np.mean(np.square(rpe_rotation)))), unit="rad")
        if rpe_rotation else unavailable("UNAVAILABLE_INSUFFICIENT_MATCHES",
                                         "NO_VALID_RPE_HORIZON_PAIR", "rad"))
    return result


def paired_aligned_metrics(run_dir: Path, suppress_dir: Path, scenario: dict):
    """Evaluate both methods on the identical set of associated GT poses."""
    unavailable_pair = {
        "status": "UNAVAILABLE",
        "reason": "PAIRED_TRAJECTORY_OR_GT_UNAVAILABLE",
        "matched_gt_count": None,
        "rmse_improvement_vs_suppress_m": None,
        "rmse_improvement_vs_suppress_pct": None,
        "p95_improvement_vs_suppress_m": None,
    }
    method_path = run_dir / "trajectory.tum"
    suppress_path = suppress_dir / "trajectory.tum"
    if (not method_path.is_file() or not suppress_path.is_file() or
            not scenario or not scenario.get("ground_truth")):
        return unavailable_pair
    gt = load_tum(Path(scenario["ground_truth"]))

    def indexed_matches(path):
        matches, _ = match_trajectories(
            load_tum(path), gt, scenario["time_association"])
        result = {}
        for pair in matches:
            key = f"{pair[1][0]:.9f}"
            if key in result:
                raise ValueError("duplicate GT point in paired trajectory")
            result[key] = pair
        return result

    method = indexed_matches(method_path)
    suppress = indexed_matches(suppress_path)
    common = sorted(set(method) & set(suppress), key=float)
    if len(common) < 3:
        return dict(unavailable_pair, reason="FEWER_THAN_THREE_COMMON_GT_POINTS",
                    matched_gt_count=len(common))
    T_Q_I_raw = scenario.get("T_Q_I")
    lever = (np.asarray(T_Q_I_raw, dtype=float).reshape(4, 4)[:3, 3]
             if T_Q_I_raw is not None else np.zeros(3))

    def errors(indexed):
        est = np.array([indexed[key][0][1] for key in common])
        gt_points = np.array([
            transform_point(np.eye(4), indexed[key][1][1],
                            indexed[key][1][2], lever) for key in common])
        est_centered = est - est.mean(axis=0)
        gt_centered = gt_points - gt_points.mean(axis=0)
        if np.linalg.matrix_rank(est_centered) < 2:
            return None
        U, _, Vt = np.linalg.svd(est_centered.T @ gt_centered)
        D = np.eye(3)
        D[2, 2] = np.linalg.det(Vt.T @ U.T)
        rotation = Vt.T @ D @ U.T
        aligned = ((rotation @ est.T).T + gt_points.mean(axis=0) -
                   rotation @ est.mean(axis=0))
        return np.linalg.norm(aligned - gt_points, axis=1)

    method_errors = errors(method)
    suppress_errors = errors(suppress)
    if method_errors is None or suppress_errors is None:
        return dict(unavailable_pair, reason="SE3_ALIGNMENT_DEGENERATE",
                    matched_gt_count=len(common))
    method_rmse = float(np.sqrt(np.mean(method_errors ** 2)))
    suppress_rmse = float(np.sqrt(np.mean(suppress_errors ** 2)))
    method_p95 = float(np.percentile(method_errors, 95))
    suppress_p95 = float(np.percentile(suppress_errors, 95))
    return {
        "status": "AVAILABLE", "reason": "",
        "matched_gt_count": len(common),
        "method_aligned_ATE_rmse_m": method_rmse,
        "suppress_aligned_ATE_rmse_m": suppress_rmse,
        "method_aligned_ATE_p95_m": method_p95,
        "suppress_aligned_ATE_p95_m": suppress_p95,
        "rmse_improvement_vs_suppress_m": suppress_rmse - method_rmse,
        "rmse_improvement_vs_suppress_pct": (
            None if suppress_rmse == 0.0 else
            100.0 * (suppress_rmse - method_rmse) / suppress_rmse),
        "rmse_improvement_pct_status": (
            "UNDEFINED_ZERO_DENOMINATOR" if suppress_rmse == 0.0 else
            "AVAILABLE"),
        "p95_improvement_vs_suppress_m": suppress_p95 - method_p95,
        "alignment": "SE3_SCALE_FIXED_ONE",
    }


def matrix_quaternion(R):
    """Stable xyzw conversion for already validated rotation matrices."""
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        index = int(np.argmax(np.diag(R)))
        if index == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
            y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
        elif index == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w, y = (R[0, 2] - R[2, 0]) / s, 0.25 * s
            x, z = (R[0, 1] + R[1, 0]) / s, (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w, z = (R[1, 0] - R[0, 1]) / s, 0.25 * s
            x, y = (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def csv_rows(path):
    if not path or not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def csv_bool(row: dict, field: str, artifact: str):
    value = str(row.get(field, "")).lower()
    if value in {"1", "true"}:
        return True
    if value in {"0", "false"}:
        return False
    raise ValueError(f"invalid {field} flag in {artifact}")


def rows_by_unique_obs(rows: list[dict], artifact: str):
    indexed = {}
    for row in rows:
        obs_id = row.get("obs_id")
        if not obs_id:
            raise ValueError(f"empty observation ID in {artifact}")
        if obs_id in indexed:
            raise ValueError(f"duplicate observation in {artifact}")
        indexed[obs_id] = row
    return indexed


def verified_observation_ledger(ledger_path: Path):
    rows = csv_rows(ledger_path)
    if not rows:
        raise ValueError("verified parent observation ledger is empty")
    by_obs = rows_by_unique_obs(rows, "parent observation ledger")
    applicable = set()
    valid_count = 0
    for obs_id, row in by_obs.items():
        valid = csv_bool(row, "valid", "parent observation ledger")
        planned = csv_bool(row, "planned", "parent observation ledger")
        valid_count += int(valid)
        if valid and planned:
            applicable.add(obs_id)
    return {"obs_ids": set(by_obs), "applicable_obs_ids": applicable,
            "fixed_valid_observations": valid_count}


def verify_final_artifacts(run_dir: Path, status: dict, cell: dict,
                           parent_cache_id: str | None = None,
                           parent_cache_dir: Path | None = None,
                           parent_payload_sha256: dict[str, str] | None = None,
                           parent_cache_manifest: dict | None = None):
    """Verify exported bytes and final mask/factor semantics before metrics."""
    manifest_path = run_dir / "run_manifest.json"
    summary_path = run_dir / "final_inference_summary.json"
    content_path = run_dir / "final_content_identity.json"
    manifest = read_json(manifest_path)
    if manifest.get("export_verification_status") != "VERIFIED":
        raise ValueError("final export is not verified")
    sealed = manifest.get("artifact_sha256")
    if not isinstance(sealed, list) or not sealed:
        raise ValueError("final artifact SHA-256 list is missing")
    sealed_names = set()
    for entry in sealed:
        if not isinstance(entry, str) or "=" not in entry:
            raise ValueError("invalid final artifact SHA-256 entry")
        name, expected = entry.split("=", 1)
        if (not name or name in {".", ".."} or "/" in name or "\\" in name or
                name in sealed_names):
            raise ValueError("invalid final artifact SHA-256 name")
        path = run_dir / name
        if not path.is_file() or file_sha256(path) != expected:
            raise ValueError(f"final artifact SHA-256 mismatch: {name}")
        sealed_names.add(name)
    required = {"final_masks.csv", "final_factor_audit.csv", "decisions.csv",
                "final_inference_summary.json", "final_content_identity.json"}
    if not required.issubset(sealed_names):
        raise ValueError("sealed final artifact set is incomplete")

    summary = read_json(summary_path)
    content = read_json(content_path)
    inference = status.get("inference_id")
    request = status.get("final_request_id")
    if (not inference or inference == "UNAVAILABLE:FINAL_ENGINE_NOT_ENTERED" or
            any(doc.get("inference_id") != inference
                for doc in (manifest, summary, content))):
        raise ValueError("final content inference identity mismatch")
    if manifest.get("final_request_id") != request:
        raise ValueError("final request identity mismatch")
    if bool(summary.get("valid_estimate")) != bool(
            status.get("valid_estimate_exported")):
        raise ValueError("final status and summary validity disagree")
    if manifest.get("canonical_mode") != cell.get("canonical_mode"):
        raise ValueError("final canonical mode mismatch")
    if (cell.get("operating_point_id") is not None and
            manifest.get("operating_point_id") != cell.get("operating_point_id")):
        raise ValueError("final operating point mismatch")
    if parent_cache_id and manifest.get("stage2_cache_id") != parent_cache_id:
        raise ValueError("final result is not bound to parent Stage-2 cache")

    masks = csv_rows(run_dir / "final_masks.csv")
    audits = csv_rows(run_dir / "final_factor_audit.csv")
    mask_by_obs = rows_by_unique_obs(masks, "final masks")
    audit_by_obs = rows_by_unique_obs(audits, "final factor audit")
    actual_final_used = None
    fixed_valid_observations = None
    if status.get("valid_estimate_exported"):
        if summary.get("factor_audit_status") != "OK":
            raise ValueError("valid final estimate lacks successful factor audit")
        if not parent_cache_id or parent_cache_dir is None:
            raise ValueError("valid final estimate lacks verified parent cache")
        parent_payload_sha256 = parent_payload_sha256 or {}
        expected_ledger_sha = parent_payload_sha256.get("observations.csv")
        parent_ledger = parent_cache_dir / "observations.csv"
        if (not expected_ledger_sha or not parent_ledger.is_file() or
                file_sha256(parent_ledger) != expected_ledger_sha):
            raise ValueError("verified parent observation ledger is unavailable")
        if ((parent_cache_manifest or {}).get(
                "observation_mapping_sha256") != expected_ledger_sha):
            raise ValueError("parent observation ledger identity mismatch")
        local_ledger = run_dir / "observations.csv"
        if (local_ledger.is_file() and
                file_sha256(local_ledger) != expected_ledger_sha):
            raise ValueError(
                "local observation ledger does not match verified parent cache")
        ledger = verified_observation_ledger(parent_ledger)
        if set(mask_by_obs) != ledger["obs_ids"]:
            raise ValueError("final mask observation domain does not match parent ledger")
        applicable_mask_ids = {
            obs_id for obs_id, row in mask_by_obs.items()
            if row.get("reason_code") != "NOT_IN_FROZEN_VALID_PLAN"}
        if applicable_mask_ids != ledger["applicable_obs_ids"]:
            raise ValueError(
                "applicable final mask domain does not match parent ledger")
        if set(audit_by_obs) != applicable_mask_ids:
            raise ValueError(
                "factor-audit observation domain does not match applicable final masks")

        verified_counts = []
        for obs_id, row in mask_by_obs.items():
            candidate = csv_bool(row, "candidate", "final masks")
            noncandidate = csv_bool(
                row, "noncandidate_reference", "final masks")
            decision_use = csv_bool(row, "decision_use", "final masks")
            final_use = csv_bool(row, "final_use", "final masks")
            fallback_use = csv_bool(row, "fallback_use", "final masks")
            if row.get("reason_code") == "NOT_IN_FROZEN_VALID_PLAN":
                if any((candidate, noncandidate, decision_use, final_use,
                        fallback_use)):
                    raise ValueError("excluded final mask has active flags")
                continue
            if candidate == noncandidate:
                raise ValueError(
                    "applicable final mask classification flags are inconsistent")
            if noncandidate:
                if decision_use or not final_use or (
                        fallback_use and not final_use):
                    raise ValueError("noncandidate final mask flags are inconsistent")
                expected_classification = "NONCANDIDATE_REFERENCE"
                semantic_expected_count = 1
            else:
                if fallback_use or (final_use and not decision_use):
                    raise ValueError("candidate final mask flags are inconsistent")
                expected_classification = (
                    ("ACCEPTED_CANDIDATE_RAW_WITH_FIXED_OFFSET" if (run_dir / "fixed_method.csv").is_file() else "ACCEPTED_CANDIDATE_RAW_WITH_LIVE_C") if final_use else
                    "SUPPRESSED_CANDIDATE")
                semantic_expected_count = int(final_use)

            audit = audit_by_obs[obs_id]
            try:
                actual = int(audit.get("final_factor_count", ""))
                expected = int(audit.get("expected_count", ""))
            except ValueError as error:
                raise ValueError("invalid final factor audit count") from error
            ok = csv_bool(audit, "ok", "final factor audit")
            if audit.get("classification") != expected_classification:
                raise ValueError("final factor audit classification mismatch")
            if (actual not in {0, 1} or expected not in {0, 1} or
                    actual != expected or expected != semantic_expected_count or
                    not ok):
                raise ValueError("final factor audit count or status mismatch")
            verified_counts.append(actual)
        actual_final_used = sum(verified_counts)
        fixed_valid_observations = ledger["fixed_valid_observations"]
    for artifact_name in sealed_names:
        if not artifact_name.endswith(".csv"):
            continue
        for row in csv_rows(run_dir / artifact_name):
            if "inference_id" in row and row.get("inference_id") != inference:
                raise ValueError(f"{artifact_name} inference identity mismatch")
    return {"manifest": manifest, "summary": summary, "content": content,
            "actual_final_used": actual_final_used,
            "fixed_valid_observations": fixed_valid_observations}


def cell_stratum(cell: dict, path: str | None = None):
    return "/".join((cell.get("canonical_mode", "UNKNOWN"),
                     cell.get("operating_point_id") or "default",
                     path or cell.get("path") or
                     cell.get("cache_namespace") or "DIRECT"))


def coverage_metrics(run_dir: Path, execution_type: str, status: dict,
                     parent_cache_dir: Path | None = None,
                     verified_final: dict | None = None):
    diagnostic_path = run_dir / "diagnostic.json"
    if execution_type == "CACHE_DIAGNOSTIC" and diagnostic_path.is_file():
        diag = read_json(diagnostic_path)
        return {key: diag[key] for key in ("candidate_use_coverage",
                                           "eligible_use_coverage",
                                           "overall_retained_fraction")}
    final_masks = csv_rows(run_dir / "final_masks.csv")
    if execution_type == "FINAL_TRAJECTORY" and final_masks:
        candidate_rows = [row for row in final_masks
                          if str(row.get("candidate", "")).lower() in {"1", "true"}]
        decision_used = sum(str(row.get("decision_use", "")).lower() in
                            {"1", "true"} for row in candidate_rows)
        decisions = csv_rows(run_dir / "decisions.csv")
        eligible_groups = {row.get("group_id") for row in decisions
                           if str(row.get("eligible", "")).lower() in {"1", "true"}}
        eligible_rows = [row for row in candidate_rows
                         if row.get("group_id") in eligible_groups]
        if status.get("valid_estimate_exported"):
            if (verified_final is None or
                    verified_final.get("actual_final_used") is None or
                    verified_final.get("fixed_valid_observations") is None):
                raise ValueError("final coverage lacks verified audit or ledger")
            overall = ratio(verified_final["actual_final_used"],
                            verified_final["fixed_valid_observations"],
                            "observation")
        else:
            overall = unavailable("UNAVAILABLE_ESTIMATION_FAILURE",
                                  "NO_VALID_FINAL_GRAPH", "observation")
        return {
            "candidate_use_coverage": ratio(decision_used, len(candidate_rows),
                                            "observation"),
            "eligible_use_coverage": ratio(decision_used, len(eligible_rows),
                                           "observation"),
            "overall_retained_fraction": overall,
        }
    if execution_type == "FINAL_TRAJECTORY":
        return {
            "candidate_use_coverage": unavailable(
                "UNAVAILABLE_UPSTREAM_FAILURE",
                "FINAL_DECISION_MASK_NOT_EMITTED", "observation"),
            "eligible_use_coverage": unavailable(
                "UNAVAILABLE_UPSTREAM_FAILURE",
                "FINAL_DECISION_MASK_NOT_EMITTED", "observation"),
            "overall_retained_fraction": unavailable(
                "UNAVAILABLE_ESTIMATION_FAILURE",
                "NO_VALID_FINAL_GRAPH", "observation"),
        }
    na_gate = unavailable("NOT_APPLICABLE_METHOD_NO_GATE",
                          "METHOD_HAS_NO_GROUP_GATE", "observation")
    if not status.get("valid_estimate_exported", (run_dir / "trajectory.tum").is_file()):
        overall = unavailable("UNAVAILABLE_ESTIMATION_FAILURE",
                              "NO_VALID_FINAL_GRAPH", "observation")
        return {"candidate_use_coverage": dict(na_gate),
                "eligible_use_coverage": dict(na_gate),
                "overall_retained_fraction": overall}
    observations_path = run_dir / "observations.csv"
    if not observations_path.is_file():
        overall = unavailable("UNAVAILABLE_ARTIFACT_MISSING",
                              "OBSERVATION_LEDGER_MISSING", "observation")
    else:
        observations = list(csv.DictReader(observations_path.open(
            newline="", encoding="utf-8")))
        valid = sum(str(row.get("valid", "")).lower() in {"1", "true"}
                    for row in observations)
        used = None
        audit_path = run_dir / "baseline_factor_audit.csv"
        final_audit_path = run_dir / "final_factor_audit.csv"
        factor_path = run_dir / "factor_metadata.csv"
        if audit_path.is_file():
            used = sum(row["final_use"] == "1" for row in csv.DictReader(
                audit_path.open(newline="", encoding="utf-8")))
        elif final_audit_path.is_file():
            used = sum(int(row.get("final_factor_count", 0)) for row in csv.DictReader(
                final_audit_path.open(newline="", encoding="utf-8")))
        elif factor_path.is_file():
            used = sum(bool(row.get("obs_id")) for row in csv.DictReader(
                factor_path.open(newline="", encoding="utf-8")))
        overall = (ratio(used, valid, "observation") if used is not None else
                   unavailable("UNAVAILABLE_FINAL_FACTOR_AUDIT",
                               "FINAL_FACTOR_MASK_MISSING", "observation"))
    return {"candidate_use_coverage": dict(na_gate),
            "eligible_use_coverage": dict(na_gate),
            "overall_retained_fraction": overall}


def bias_correction_metrics(run_dir: Path, scenario: dict,
                            execution_type: str = "",
                            parent_cache_dir: Path | None = None):
    names = ("candidate_bias_field_RMSE", "accepted_bias_field_RMSE",
             "bad_correction_rate", "good_correction_rejection_rate")
    if scenario.get("bias_truth_scope", "NONE") != "TOTAL_SYNTHETIC_LATENT":
        value = unavailable("UNAVAILABLE_INCOMPLETE_BIAS_TRUTH",
                            "BIAS_TRUTH_IS_NOT_TOTAL_SYNTHETIC_LATENT")
        return {name: dict(value) for name in names}
    truth_path = scenario.get("bias_truth")
    provenance = scenario.get("bias_truth_provenance")
    epsilon = scenario.get("epsilon_bad_m")
    if not truth_path or not provenance or epsilon is None:
        value = unavailable("UNAVAILABLE_BIAS_TRUTH_PROVENANCE",
                            "TRUTH_PATH_PROVENANCE_OR_EPSILON_MISSING")
        return {name: dict(value) for name in names}
    epsilon = float(epsilon)
    if not math.isfinite(epsilon) or epsilon < 0:
        raise ValueError("epsilon_bad_m must be finite and nonnegative")
    truth_rows = list(csv.DictReader(Path(truth_path).open(
        newline="", encoding="utf-8")))
    truth = {}
    for row in truth_rows:
        obs_id = row.get("obs_id")
        value = row.get("total_bias_m", row.get("bias_m"))
        if not obs_id or value is None or obs_id in truth:
            raise ValueError("bias truth rows require unique obs_id and total_bias_m")
        truth[obs_id] = float(value)

    decision_estimate = {}
    decision_accepted = set()
    final_estimate = {}
    final_accepted = set()
    stage1 = run_dir / "stage1_observation_bias.csv"
    if stage1.is_file():
        for row in csv.DictReader(stage1.open(newline="", encoding="utf-8")):
            if "candidate" in row and str(row["candidate"]).lower() not in {
                    "1", "true"}:
                continue
            decision_estimate[row["obs_id"]] = float(row["bias_m"])
            decision_accepted.add(row["obs_id"])
        final_estimate = dict(decision_estimate)
        final_accepted = set(decision_accepted)
    else:
        stage2_root = parent_cache_dir or run_dir
        partition_path = stage2_root / "partition.json"
        segments_path = stage2_root / "segments.csv"
        if partition_path.is_file() and segments_path.is_file():
            partition = read_json(partition_path)
            amplitudes = {row["segment_id"]: float(row["amplitude_m"])
                          for row in csv_rows(segments_path)}
            ordinal_to_segment = {}
            for segment in partition.get("segments", []):
                segment_id = segment["segment_id"]
                ordinal_to_segment[str(segment["segment_ordinal"])] = segment_id
                if segment_id not in amplitudes:
                    continue
                for obs_id in segment.get("obs_ids", []):
                    decision_estimate[str(obs_id)] = amplitudes[segment_id]
            if execution_type == "CACHE_DIAGNOSTIC":
                diag = read_json(run_dir / "diagnostic.json") or {}
                used_groups = {row.get("group_id") for row in diag.get("decisions", [])
                               if row.get("decision") == "USE"}
                group_rows = csv_rows(stage2_root / "groups.csv")
                used_ordinals = set()
                for row in group_rows:
                    if row.get("group_id") in used_groups:
                        used_ordinals.update(item for item in
                                             row.get("segment_ordinals", "").split(";")
                                             if item)
                used_segments = {ordinal_to_segment[item] for item in used_ordinals
                                 if item in ordinal_to_segment}
                for segment in partition.get("segments", []):
                    if segment["segment_id"] in used_segments:
                        decision_accepted.update(
                            str(obs) for obs in segment.get("obs_ids", []))
                final_estimate = dict(decision_estimate)
                final_accepted = set(decision_accepted)
            elif execution_type == "FINAL_TRAJECTORY":
                masks = csv_rows(run_dir / "final_masks.csv")
                decision_accepted = {
                    row["obs_id"] for row in masks
                    if str(row.get("candidate", "")).lower() in {"1", "true"}
                    and str(row.get("decision_use", "")).lower() in {"1", "true"}}
                final_accepted = {
                    row["obs_id"] for row in masks
                    if str(row.get("candidate", "")).lower() in {"1", "true"}
                    and str(row.get("final_use", "")).lower() in {"1", "true"}}
                final_amplitudes = {row["segment_id"]: float(row["amplitude_m"])
                                    for row in csv_rows(run_dir / "segment_bias.csv")}
                if (run_dir / "fixed_compensations.csv").is_file():
                    final_amplitudes = {row["segment_id"]: float(row["delta_c_fixed_m"])
                        for row in csv_rows(run_dir / "fixed_compensations.csv") if row["final_use"] == "1"}
                segment_by_obs = {str(obs): segment["segment_id"]
                                  for segment in partition.get("segments", [])
                                  for obs in segment.get("obs_ids", [])}
                missing_final = []
                for obs_id in final_accepted:
                    segment_id = segment_by_obs.get(obs_id)
                    if segment_id in final_amplitudes:
                        final_estimate[obs_id] = final_amplitudes[segment_id]
                    else:
                        missing_final.append(obs_id)
                if missing_final:
                    final_estimate = {}
            else:
                decision_accepted = set(decision_estimate)
                final_estimate = dict(decision_estimate)
                final_accepted = set(decision_accepted)
    if (not decision_estimate or
            any(obs_id not in truth for obs_id in decision_estimate)):
        value = unavailable("UNAVAILABLE_ARTIFACT_MISSING",
                            "COMPLETE_BIAS_ESTIMATE_OR_TRUTH_MAPPING_MISSING")
        return {name: dict(value) for name in names}

    decision_errors = {obs_id: abs(value - truth[obs_id])
                       for obs_id, value in decision_estimate.items()}
    decision_accepted_errors = [
        error for obs_id, error in decision_errors.items()
        if obs_id in decision_accepted]
    candidate_rmse = math.sqrt(sum(error * error for error in
                                   decision_errors.values()) /
                               len(decision_errors))
    decision_accepted_rmse = (
        metric(math.sqrt(sum(error * error for error in
                             decision_accepted_errors) /
                         len(decision_accepted_errors)),
               denominator=len(decision_accepted_errors), unit="m")
        if decision_accepted_errors else unavailable(
            "UNDEFINED_ZERO_DENOMINATOR", "NO_DECISION_USE_CORRECTIONS", "m"))
    decision_bad_rate = ratio(
        sum(error > epsilon for error in decision_accepted_errors),
        len(decision_accepted_errors), "observation")
    good = {obs_id for obs_id, error in decision_errors.items()
            if error <= epsilon}
    decision_good_rejection = ratio(
        sum(obs_id not in decision_accepted for obs_id in good), len(good),
        "observation")

    if execution_type == "FINAL_TRAJECTORY":
        if final_accepted and not final_estimate:
            final_rmse = unavailable(
                "UNAVAILABLE_FINAL_AMPLITUDE_MAPPING",
                "ACCEPTED_OBSERVATION_HAS_NO_VERIFIED_FINAL_C_STATE", "m")
            final_bad_rate = unavailable(
                "UNAVAILABLE_FINAL_AMPLITUDE_MAPPING",
                "ACCEPTED_OBSERVATION_HAS_NO_VERIFIED_FINAL_C_STATE",
                "observation")
        elif not final_accepted:
            final_rmse = unavailable(
                "UNDEFINED_ZERO_DENOMINATOR", "NO_FINAL_ACCEPTED_CORRECTIONS", "m")
            final_bad_rate = ratio(0, 0, "observation")
        elif (set(final_estimate) != final_accepted or
              any(obs_id not in truth for obs_id in final_estimate)):
            final_rmse = unavailable(
                "UNAVAILABLE_FINAL_AMPLITUDE_MAPPING",
                "FINAL_C_STATE_OR_TRUTH_MAPPING_INCOMPLETE", "m")
            final_bad_rate = unavailable(
                "UNAVAILABLE_FINAL_AMPLITUDE_MAPPING",
                "FINAL_C_STATE_OR_TRUTH_MAPPING_INCOMPLETE", "observation")
        else:
            final_errors = [abs(final_estimate[obs_id] - truth[obs_id])
                            for obs_id in sorted(final_accepted)]
            final_rmse = metric(
                math.sqrt(sum(error * error for error in final_errors) /
                          len(final_errors)), denominator=len(final_errors), unit="m")
            final_bad_rate = ratio(sum(error > epsilon for error in final_errors),
                                   len(final_errors), "observation")
    else:
        final_rmse = decision_accepted_rmse
        final_bad_rate = decision_bad_rate

    result = {
        "candidate_bias_field_RMSE": metric(
            candidate_rmse, denominator=len(decision_errors), unit="m"),
        "accepted_bias_field_RMSE": final_rmse,
        "bad_correction_rate": final_bad_rate,
        "good_correction_rejection_rate": decision_good_rejection,
        "decision_time": {
            "candidate_bias_field_RMSE": metric(
                candidate_rmse, denominator=len(decision_errors), unit="m"),
            "accepted_bias_field_RMSE": decision_accepted_rmse,
            "bad_correction_rate": decision_bad_rate,
            "good_correction_rejection_rate": decision_good_rejection,
        },
        "final_time": {
            "accepted_bias_field_RMSE": final_rmse,
            "bad_correction_rate": final_bad_rate,
        },
    }
    return result


def score_and_support_diagnostics(run_dir: Path,
                                  parent_cache_dir: Path | None = None):
    score_tables = {}
    for name in ("scores_decision.csv", "scores_recovery_attempt.csv",
                 "scores_final.csv"):
        path = run_dir / name
        if path.is_file():
            score_tables[name] = list(csv.DictReader(path.open(
                newline="", encoding="utf-8")))
    diagnostic = run_dir / "diagnostic.json"
    if diagnostic.is_file():
        score_tables["cache_policy_decisions"] = read_json(
            diagnostic).get("decisions", [])
    support_root = (parent_cache_dir if parent_cache_dir and
                    not (run_dir / "segments.csv").is_file() else run_dir)
    if parent_cache_dir and (parent_cache_dir / "scores_decision.csv").is_file():
        score_tables.setdefault("scores_decision.csv", csv_rows(
            parent_cache_dir / "scores_decision.csv"))
    segments_path = support_root / "segments.csv"
    groups_path = support_root / "groups.csv"
    segments = (list(csv.DictReader(segments_path.open(
        newline="", encoding="utf-8"))) if segments_path.is_file() else [])
    groups = (list(csv.DictReader(groups_path.open(
        newline="", encoding="utf-8"))) if groups_path.is_file() else [])
    return {
        "score_tables": score_tables,
        "score_semantics": "DECISION_FINAL_SEPARATE_POSTFIT_RESIDUAL_NOT_BIAS_TRUTH",
        "support": {
            "segment_count": len(segments), "group_count": len(groups),
            "short_segment_count": sum(str(row.get(
                "short_support_debug", "")).lower() in {"1", "true"}
                                       for row in segments),
            "boundary_segment_count": sum(str(row.get(
                "boundary", "")).lower() in {"1", "true"}
                                          for row in segments),
            "segments": segments, "groups": groups,
        },
    }


def range_reference_metrics(run_dir: Path, scenario: dict, parent_dir=None):
    """Evaluation-only noisy geometric agreement, never latent bias truth.

    Reference CSV: obs_id, geometric_range_m, fixed_beta_m. The manifest must
    bind its hash and independent frame/point/time/beta provenance explicitly.
    No fitted trajectory/residual is accepted as a geometric reference.
    """
    na = unavailable("UNAVAILABLE_REFERENCE_PROVENANCE",
                     "INDEPENDENT_GEOMETRY_BETA_TIME_REFERENCE_MISSING", "m")
    if not run_dir or not scenario.get("range_reference_csv"):
        return na
    required = ("range_reference_sha256", "range_reference_provenance")
    if any(not scenario.get(key) for key in required):
        raise ValueError("range reference lacks hash or provenance")
    provenance = scenario["range_reference_provenance"]
    if not isinstance(provenance, dict) or any(not provenance.get(key) for key in
            ("independent_geometry", "frame", "point", "time", "fixed_beta")):
        raise ValueError("range reference provenance is incomplete")
    path = Path(scenario["range_reference_csv"])
    if file_sha256(path) != scenario["range_reference_sha256"]:
        raise ValueError("range reference hash mismatch")
    reference = {}
    for row in csv_rows(path):
        key = row["obs_id"]
        if key in reference:
            raise ValueError("duplicate reference obs_id")
        values = float(row["geometric_range_m"]), float(row["fixed_beta_m"])
        if not all(math.isfinite(x) for x in values) or values[0] < 0:
            raise ValueError("invalid geometric reference")
        reference[key] = values
    source = run_dir / "observations.csv"
    if not source.is_file() and parent_dir:
        source = parent_dir / "observations.csv"
    observations = csv_rows(source)
    masks = {r["obs_id"]: r for r in csv_rows(run_dir / "final_masks.csv")}
    corrections = {r["segment_id"]: float(r["delta_c_fixed_m"])
                   for r in csv_rows(run_dir / "fixed_compensations.csv")}
    if not corrections:
        corrections = {r["segment_id"]: float(r["amplitude_m"])
                       for r in csv_rows(run_dir / "segment_bias.csv")}
    raw_all, before, after = [], [], []
    seen = set()
    for row in observations:
        key = row["obs_id"]
        if key in seen:
            raise ValueError("duplicate observation identity")
        seen.add(key)
        if key not in reference:
            continue
        geometry, beta = reference[key]
        raw = float(row["raw_z_m"])-beta-geometry
        if not math.isfinite(raw):
            raise ValueError("nonfinite raw reference error")
        raw_all.append(abs(raw))
        mask = masks.get(key, {})
        if mask.get("candidate") == "1" and mask.get("final_use") == "1":
            correction = corrections.get(mask["segment_id"])
            if correction is None or not math.isfinite(correction):
                raise ValueError("restored observation lacks correction")
            before.append(abs(raw)); after.append(abs(raw-correction))
    result = {"semantics": "NOISY_GEOMETRIC_MEASUREMENT_REFERENCE_NOT_BIAS_TRUTH",
              "reference_sha256": scenario["range_reference_sha256"],
              "matched_raw_count": len(raw_all), "restored_pair_count": len(before)}
    for values, name in ((raw_all,"raw_pool"),(before,"restored_before"),(after,"restored_after")):
        result.update(summarize_errors(np.asarray(values),name) if values else
                      {name+"_rmse_m": unavailable("UNDEFINED_ZERO_COVERAGE", "NO_MATCHED_OBSERVATIONS", "m")})
    return result


def validate_evaluation_manifest(path: Path):
    with path.open(encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict) or value.get("schema") != "uifgo_t09_evaluation_v1":
        raise ValueError("evaluation manifest schema mismatch")
    scenarios = value.get("run_units", {})
    if not isinstance(scenarios, dict):
        raise ValueError("evaluation run_units must be a mapping")
    for scenario in scenarios.values():
        if scenario.get("ground_truth"):
            gt = Path(scenario["ground_truth"])
            if not gt.is_absolute():
                gt = (path.parent / gt).resolve()
                scenario["ground_truth"] = str(gt)
            required = ("ground_truth_sha256", "units", "time_base",
                        "T_G_E", "T_Q_I", "frame_point_provenance",
                        "time_association")
            if any(key not in scenario for key in required):
                raise ValueError("GT evaluation scenario lacks required provenance")
            if scenario["units"] != "m_s_rad":
                raise ValueError("GT evaluation units must be m_s_rad")
            if file_sha256(gt) != scenario["ground_truth_sha256"]:
                raise ValueError("ground-truth source hash mismatch")
        if scenario.get("range_reference_csv"):
            reference = Path(scenario["range_reference_csv"])
            if not reference.is_absolute():
                scenario["range_reference_csv"] = str((path.parent / reference).resolve())
        if scenario.get("bias_truth"):
            truth = Path(scenario["bias_truth"])
            if not truth.is_absolute():
                scenario["bias_truth"] = str((path.parent / truth).resolve())
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--batch-manifest", required=True)
    parser.add_argument("--evaluation-manifest", required=True)
    parser.add_argument("--output", default=None)
    args = parser.parse_args()
    try:
        batch_path = Path(args.batch_manifest).resolve()
        batch = read_json(batch_path)
        if batch.get("schema") not in {"uifgo_t09_batch_manifest_v1",
                                      "uifgo_t09_batch_manifest_v2"}:
            raise ValueError("batch manifest schema mismatch")
        if batch.get("status") not in {"COMPLETE", "COMPLETE_WITH_RUN_FAILURES"}:
            raise ValueError("batch is not terminal")
        if any(not cell_is_comparable(cell) for cell in batch.get("cells", [])):
            raise ValueError("COMPARABILITY_INVALID cells are quarantined from evaluation")
        evaluation = validate_evaluation_manifest(Path(args.evaluation_manifest).resolve())
        output_path = Path(args.output).resolve() if args.output else batch_path.parent / "evaluation.json"
        run_metrics = []
        trajectory_outcomes: dict[str, dict[str, bool]] = {}
        automatic_outcomes: dict[str, bool] = {}
        diagnostic_outcomes: dict[str, dict[str, bool]] = {}
        score_groups: dict[str, set[tuple[str, str]]] = {}
        unavailable_score_groups: dict[str, set[tuple[str, str]]] = {}
        final_outcomes: dict[str, dict[str, bool]] = {}
        stage4_entries: dict[str, dict[str, bool]] = {}
        recovery_outcomes: dict[str, dict[str, bool]] = {}
        recovery_solve_outcomes: dict[str, dict[str, bool]] = {}
        recovery_audit_outcomes: dict[str, dict[str, bool]] = {}
        fallback_attempt_outcomes: dict[str, dict[str, bool]] = {}
        fallback_success_outcomes: dict[str, dict[str, bool]] = {}
        inference_by_request: dict[str, set[str]] = {}

        def record(table, key, run_unit_id, outcome):
            values = table.setdefault(key, {})
            values[run_unit_id] = bool(values.get(run_unit_id, False) or outcome)

        def fraction(values):
            return ratio(sum(values.values()), len(values), "run_unit")

        for cell in batch["cells"]:
            mode = cell["canonical_mode"]
            execution = cell["execution_type"]
            run_unit_id = cell["run_unit_id"]
            path_name = (cell.get("path") or cell.get("cache_namespace") or
                         "DIRECT_COMMON_PREPARATION")
            stratum = cell_stratum(cell, path_name)
            run_dir = Path(cell["run_directory"]) if cell.get("run_directory") else None
            status = read_json(run_dir / "run_status.json") if run_dir and (run_dir / "run_status.json").is_file() else {}
            parent_cache_dir = None
            parent_payload_sha256 = {}
            parent_cache_manifest = None
            verified_parent_cache_id = None
            if cell.get("parent_cache_manifest"):
                parent_manifest_path = Path(cell["parent_cache_manifest"])
                parent = read_json(parent_manifest_path)
                if parent.get("schema") != "uifgo_t09_stage2_cache_v2":
                    raise ValueError("cell parent cache schema mismatch")
                try:
                    computed_cache_id = stage2_cache_identity(parent)
                except (KeyError, TypeError, ValueError) as error:
                    raise ValueError(
                        "cell parent cache canonical identity input is invalid") from error
                if parent.get("cache_id") != computed_cache_id:
                    raise ValueError("cell parent cache content identity mismatch")
                if computed_cache_id != cell.get("cache_id"):
                    raise ValueError("cell parent cache identity mismatch")
                for payload in parent.get("payloads", []):
                    name = payload.get("name", "")
                    if (not name or name in parent_payload_sha256 or
                            name in {".", ".."} or "/" in name or "\\" in name):
                        raise ValueError("invalid cell parent cache payload name")
                    expected_sha = payload.get("sha256")
                    path = parent_manifest_path.parent / name
                    if not path.is_file() or file_sha256(path) != expected_sha:
                        raise ValueError("cell parent cache payload mismatch")
                    parent_payload_sha256[name] = expected_sha
                parent_cache_dir = parent_manifest_path.parent
                parent_cache_manifest = parent
                verified_parent_cache_id = computed_cache_id
            terminal_failure = cell["status"] in {"COMPLETE_WITH_RUN_FAILURE",
                                                   "PARENT_CACHE_UNAVAILABLE",
                                                   "COMPARABILITY_INVALID"}
            trajectory_missing = (execution in {"BASELINE_TRAJECTORY", "STAGE1_TRAJECTORY",
                                  "AUTOMATIC_STAGE2_TRAJECTORY", "FINAL_TRAJECTORY"}
                                  and (run_dir is None or not (run_dir / "trajectory.tum").is_file()))
            if execution in {"BASELINE_TRAJECTORY", "STAGE1_TRAJECTORY",
                             "AUTOMATIC_STAGE2_TRAJECTORY", "FINAL_TRAJECTORY"}:
                record(trajectory_outcomes, stratum, run_unit_id,
                       terminal_failure or trajectory_missing)
            if (mode == "structured_debias" and execution in {
                    "CACHE_PRODUCER", "AUTOMATIC_STAGE2_TRAJECTORY"} and
                    cell.get("cache_namespace") != "FIXED_PARTITION_DEBUG"):
                automatic_outcomes[run_unit_id] = bool(
                    automatic_outcomes.get(run_unit_id, False) or
                    terminal_failure or not cell.get("cache_id"))
            if execution == "CACHE_DIAGNOSTIC":
                diagnostic_doc = (read_json(run_dir / "diagnostic.json")
                                  if run_dir and (run_dir / "diagnostic.json").is_file()
                                  else None)
                path = ((diagnostic_doc or {}).get("cache_namespace") or
                        cell.get("cache_namespace") or path_name or
                        "UPSTREAM_UNKNOWN")
                path_key = cell_stratum(cell, path)
                unavailable_cell = terminal_failure or diagnostic_doc is None
                record(diagnostic_outcomes, path_key, run_unit_id,
                       unavailable_cell)
                decisions = (diagnostic_doc or {}).get("decisions", [])
                group_set = score_groups.setdefault(path_key, set())
                unavailable_set = unavailable_score_groups.setdefault(
                    path_key, set())
                for index, row in enumerate(decisions):
                    group_key = (run_unit_id, str(row.get("group_id", index)))
                    group_set.add(group_key)
                    if row.get("reason") == "SUPPRESS_REQUIRED_SCORE_UNAVAILABLE":
                        unavailable_set.add(group_key)
            if execution == "FINAL_TRAJECTORY":
                record(final_outcomes, stratum, run_unit_id,
                       terminal_failure or trajectory_missing)
                verified_final = None
                has_bundle = bool(run_dir and
                                  (run_dir / "run_manifest.json").is_file() and
                                  (run_dir / "final_inference_summary.json").is_file() and
                                  (run_dir / "final_content_identity.json").is_file())
                if status.get("valid_estimate_exported") and not has_bundle:
                    raise ValueError("valid final estimate lacks identity bundle")
                if has_bundle:
                    verified_final = verify_final_artifacts(
                        run_dir, status, cell, verified_parent_cache_id,
                        parent_cache_dir, parent_payload_sha256,
                        parent_cache_manifest)
                    final_summary = verified_final["summary"]
                    record(stage4_entries, stratum, run_unit_id, True)
                else:
                    final_summary = {}
            else:
                verified_final = None
                final_summary = {}
            recovery_execution = final_summary.get("recovery_attempt_execution_status")
            if recovery_execution and recovery_execution != "NOT_RUN":
                solve_failed = (recovery_execution != "SUCCEEDED" or
                                final_summary.get("recovery_attempt_solver_status") != "CONVERGED")
                audit_status = final_summary.get("recovery_attempt_acceptance_audit_status")
                audit_failed = str(audit_status).startswith("FAILED")
                record(recovery_outcomes, stratum, run_unit_id,
                       solve_failed or audit_failed)
                record(recovery_solve_outcomes, stratum, run_unit_id,
                       solve_failed)
                record(recovery_audit_outcomes, stratum, run_unit_id,
                       audit_failed)
            fallback_count = int(final_summary.get("fallback_attempt_count", 0) or 0)
            if fallback_count:
                record(fallback_attempt_outcomes, stratum, run_unit_id, True)
                record(fallback_success_outcomes, stratum, run_unit_id,
                       status.get("status") == "FALLBACK_OK")
            if status.get("final_request_id"):
                inference_by_request.setdefault(status["final_request_id"], set()).add(
                    status.get("inference_id", "UNAVAILABLE"))
            scenario = evaluation["run_units"].get(cell["run_unit_id"], {})
            if run_dir:
                trajectory = trajectory_metrics(run_dir, scenario)
                coverage = coverage_metrics(run_dir, execution, status,
                                            parent_cache_dir, verified_final)
            else:
                trajectory = trajectory_metrics(Path("/nonexistent"), scenario)
                coverage = {"candidate_use_coverage": unavailable(
                                "UNAVAILABLE_UPSTREAM_FAILURE", "CELL_NOT_EXECUTED"),
                            "eligible_use_coverage": unavailable(
                                "UNAVAILABLE_UPSTREAM_FAILURE", "CELL_NOT_EXECUTED"),
                            "overall_retained_fraction": unavailable(
                                "UNAVAILABLE_ESTIMATION_FAILURE", "CELL_NOT_EXECUTED")}
            bias_metrics = (bias_correction_metrics(run_dir, scenario, execution,
                                                     parent_cache_dir)
                            if run_dir else {
                                name: unavailable("UNAVAILABLE_ARTIFACT_MISSING",
                                                  "CELL_NOT_EXECUTED")
                                for name in (
                                    "candidate_bias_field_RMSE",
                                    "accepted_bias_field_RMSE",
                                    "bad_correction_rate",
                                    "good_correction_rejection_rate")})
            score_support = (score_and_support_diagnostics(run_dir,
                                                           parent_cache_dir)
                             if run_dir else {"score_tables": {},
                                               "support": {},
                                               "score_semantics":
                                               "CELL_NOT_EXECUTED"})
            covariance_path = (run_dir / "covariance_status.json") if run_dir else None
            covariance_doc = (read_json(covariance_path)
                              if covariance_path and covariance_path.is_file() else {})
            run_metrics.append({
                "cell_id": cell["cell_id"], "run_unit_id": cell["run_unit_id"],
                "mode": mode, "execution_type": execution,
                "operating_point_id": cell.get("operating_point_id"),
                "path": path_name, "producer_id": cell.get("producer_id"),
                "cache_id": cell.get("cache_id"),
                "final_request_id": status.get("final_request_id"),
                "inference_id": status.get("inference_id"),
                "aggregate_stratum": stratum,
                "cell_status": cell["status"], "trajectory": trajectory,
                "run_directory": str(run_dir) if run_dir else None,
                "coverage": coverage, "bias_correction": bias_metrics,
                "residual_score": score_support["score_tables"],
                "residual_score_semantics": score_support["score_semantics"],
                "support": score_support["support"],
                "trajectory_harm": unavailable(
                    "UNAVAILABLE_UNPAIRED_USE_SUPPRESS_TRAJECTORIES",
                    "REQUIRES_MATCHED_USE_AND_SUPPRESS_FINAL_REQUESTS", "m"),
                "measurement_reference_agreement": range_reference_metrics(
                    run_dir, scenario, parent_cache_dir),
                "covariance": {"status": covariance_doc.get(
                    "status", status.get("covariance_status",
                                         "NOT_APPLICABLE_OR_UNAVAILABLE")),
                    "reason": covariance_doc.get("reason", ""),
                    "source": covariance_doc.get(
                        "source", "FULL_FINAL_GRAPH_MARGINAL_COVARIANCE")},
                "resources": {
                    "measured": {
                        "wall_seconds": (cell.get("attempts") or [{}])[-1].get(
                            "measured_wall_seconds"),
                        "peak_rss_kib": (cell.get("attempts") or [{}])[-1].get(
                            "measured_peak_rss_kib")},
                    "reported_or_estimated": {
                        "stage1_seconds": status.get("stage1_seconds"),
                        "stage2_seconds": status.get("stage2_seconds"),
                        "stage3_score_seconds": status.get(
                            "stage3_decision_score_seconds"),
                        "runner_total_wall_seconds": status.get("elapsed_seconds"),
                        "runner_total_wall_semantics": status.get(
                            "elapsed_seconds_semantics"),
                        "stage4_seconds": status.get("stage4_engine_seconds"),
                        "stage4_decision_seconds": status.get(
                            "stage4_decision_seconds"),
                        "stage4_recovery_refit_seconds": status.get(
                            "stage4_recovery_refit_seconds"),
                        "score_overhead_seconds": status.get(
                            "stage4_final_score_seconds"),
                        "fallback_seconds": status.get(
                            "stage4_fallback_seconds"),
                        "stage4_covariance_seconds": status.get(
                            "stage4_covariance_seconds"),
                        "artifact_export_seconds": status.get(
                            "inference_artifact_export_seconds")}},
            })
        paired_improvements = []
        suppress_by_unit_path = {}
        for row in run_metrics:
            if (row["mode"] == "suppress_all" and
                    row["execution_type"] == "FINAL_TRAJECTORY"):
                suppress_by_unit_path[(row["run_unit_id"], row["path"])] = row
        for row in run_metrics:
            if row["execution_type"] != "FINAL_TRAJECTORY":
                continue
            suppress = suppress_by_unit_path.get((row["run_unit_id"], row["path"]))
            scenario = evaluation["run_units"].get(row["run_unit_id"], {})
            if suppress and row.get("run_directory") and suppress.get("run_directory"):
                paired = paired_aligned_metrics(
                    Path(row["run_directory"]), Path(suppress["run_directory"]),
                    scenario)
            else:
                paired = {
                    "status": "UNAVAILABLE", "reason": "SUPPRESS_PAIR_UNAVAILABLE",
                    "matched_gt_count": None,
                    "rmse_improvement_vs_suppress_m": None,
                    "rmse_improvement_vs_suppress_pct": None,
                    "p95_improvement_vs_suppress_m": None,
                }
            row["paired_vs_suppress"] = paired
            paired_improvements.append({
                "cell_id": row["cell_id"], "run_unit_id": row["run_unit_id"],
                "mode": row["mode"], "path": row["path"], **paired})

        aggregate = {
            "trajectory_failure_fraction": {
                key: fraction(values) for key, values in trajectory_outcomes.items()},
            "automatic_discovery_failure_fraction": fraction(automatic_outcomes),
            "diagnostic_cell_unavailable_fraction": ratio(
                sum(sum(values.values()) for values in diagnostic_outcomes.values()),
                sum(len(values) for values in diagnostic_outcomes.values()),
                "run_unit"),
            "diagnostic_cell_unavailable_fraction_by_policy_path": {
                key: fraction(values) for key, values in diagnostic_outcomes.items()},
            "score_unavailable_group_fraction": {
                key: ratio(len(unavailable_score_groups.get(key, set())),
                           len(values), "group")
                for key, values in score_groups.items()},
            "final_estimation_failure_fraction": {
                key: fraction(values) for key, values in final_outcomes.items()},
            "recovery_failure_fraction": {
                key: fraction(values) for key, values in recovery_outcomes.items()},
            "recovery_solve_failure_fraction": {
                key: fraction(values)
                for key, values in recovery_solve_outcomes.items()},
            "recovery_final_audit_failure_fraction": {
                key: fraction(values)
                for key, values in recovery_audit_outcomes.items()},
            "fallback_attempt_fraction": {
                key: ratio(sum(fallback_attempt_outcomes.get(key, {}).values()),
                           len(values), "run_unit")
                for key, values in stage4_entries.items()},
            "fallback_success_fraction": {
                key: fraction(fallback_success_outcomes.get(key, {}))
                for key in fallback_attempt_outcomes},
            "rerun_disagreement": {request: sorted(ids) for request, ids in inference_by_request.items()
                                     if len(ids) > 1},
            "independent_sample_unit": "run_unit_id",
        }
        result = {"schema": "uifgo_t09_evaluation_result_v1",
                  "batch_status": batch["status"], "run_metrics": run_metrics,
                  "paired_improvements_vs_suppress": paired_improvements,
                  "aggregate": aggregate}
        atomic_json(output_path, result)
        csv_path = output_path.with_suffix(".csv")
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["cell_id", "run_unit_id", "mode", "execution_type",
                             "cell_status", "raw_frame_ATE_rmse_m",
                             "aligned_ATE_rmse_m", "overall_retained_fraction"])
            for row in run_metrics:
                writer.writerow([row["cell_id"], row["run_unit_id"], row["mode"],
                                 row["execution_type"], row["cell_status"],
                                 row["trajectory"].get("raw_frame_ATE_rmse_m", {}).get("value"),
                                 row["trajectory"].get("aligned_ATE_rmse_m", {}).get("value"),
                                 row["coverage"]["overall_retained_fraction"].get("value")])
        return 0
    except Exception as error:
        print(f"evaluate_runs ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
