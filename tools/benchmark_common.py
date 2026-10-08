#!/usr/bin/env python3
"""Shared data contract, metrics, and report helpers for dataset benchmarks."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import platform
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


ROOT = Path(__file__).resolve().parents[1]
CACHE_SCHEMA = "uwb-imu-benchmark-cache/v1"
IMU_FIELDS = ("t", "ax", "ay", "az", "gx", "gy", "gz")
UWB_FIELDS = (
    "t", "source_message", "source_range", "tag_id", "anchor_id",
    "range", "range_raw", "sigma", "lever_x", "lever_y", "lever_z",
    "anchor_x", "anchor_y", "anchor_z", "valid")
GT_FIELDS = ("t", "px", "py", "pz", "qx", "qy", "qz", "qw", "tag_id")
ANCHOR_FIELDS = ("anchor_id", "x", "y", "z")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_identity(path: Path) -> dict:
    def command(*args):
        return subprocess.check_output(
            ["git", "-C", str(path), *args], text=True,
            stderr=subprocess.DEVNULL).strip()
    try:
        return {"commit": command("rev-parse", "HEAD"),
                "dirty": bool(command("status", "--porcelain"))}
    except (OSError, subprocess.CalledProcessError):
        return {"commit": "UNKNOWN", "dirty": None}


def system_identity() -> dict:
    cpu = "UNKNOWN"
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                cpu = line.split(":", 1)[1].strip()
                break
    except OSError:
        pass
    return {"platform": platform.platform(), "python": platform.python_version(),
            "cpu": cpu, "omp_num_threads": os.environ.get("OMP_NUM_THREADS")}


def write_csv(path: Path, fields: Sequence[str], rows: Iterable[dict]) -> int:
    path.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
            count += 1
    return count


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def csv_duration(path: Path) -> float:
    rows = read_csv(path)
    if len(rows) < 2:
        return 0.0
    times = [float(row["t"]) for row in rows]
    return max(times) - min(times)


def stamp_seconds(stamp) -> float:
    if hasattr(stamp, "sec"):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return float(stamp.secs) + float(stamp.nsecs) * 1e-9


def header_time(message, fallback_ns: int) -> float:
    if hasattr(message, "header") and hasattr(message.header, "stamp"):
        value = stamp_seconds(message.header.stamp)
        if value > 0:
            return value
    if hasattr(message, "stamp"):
        value = stamp_seconds(message.stamp)
        if value > 0:
            return value
    return fallback_ns * 1e-9


def quaternion_dict(q) -> tuple[float, float, float, float]:
    return float(q.x), float(q.y), float(q.z), float(q.w)


def pose_row(t: float, position, quaternion, tag_id: int) -> dict:
    qx, qy, qz, qw = quaternion_dict(quaternion)
    return {"t": t, "px": float(position.x), "py": float(position.y),
            "pz": float(position.z), "qx": qx, "qy": qy, "qz": qz,
            "qw": qw, "tag_id": int(tag_id)}


def vector(values) -> np.ndarray:
    return np.asarray(values, dtype=float)


def quat_xyzw(row: dict) -> np.ndarray:
    return np.array([row["qx"], row["qy"], row["qz"], row["qw"]], dtype=float)


def apply_tag_lever(position: np.ndarray, quat: np.ndarray,
                    lever: Sequence[float]) -> np.ndarray:
    return position + Rotation.from_quat(quat).apply(vector(lever))


@dataclass
class Trajectory:
    time: np.ndarray
    position: np.ndarray
    quaternion: np.ndarray

    def validate(self):
        if len(self.time) < 2 or self.position.shape != (len(self.time), 3):
            raise ValueError("trajectory has insufficient or malformed samples")
        if self.quaternion.shape != (len(self.time), 4):
            raise ValueError("trajectory quaternion shape is invalid")
        if not np.isfinite(self.time).all() or not np.isfinite(self.position).all() \
                or not np.isfinite(self.quaternion).all():
            raise ValueError("trajectory contains non-finite values")
        if np.any(np.diff(self.time) <= 0):
            raise ValueError("trajectory timestamps are not strictly increasing")
        norms = np.linalg.norm(self.quaternion, axis=1)
        if np.any(norms < 1e-8):
            raise ValueError("trajectory contains a zero quaternion")
        self.quaternion[:] = self.quaternion / norms[:, None]


def load_tum(path: Path) -> Trajectory:
    data = np.loadtxt(path, ndmin=2)
    if data.shape[1] != 8:
        raise ValueError(f"{path} is not TUM t xyz qxyzw")
    trajectory = Trajectory(data[:, 0], data[:, 1:4], data[:, 4:8])
    trajectory.validate()
    return trajectory


def load_gt(path: Path, tag_id: int) -> Trajectory:
    rows = [row for row in read_csv(path) if int(row["tag_id"]) == int(tag_id)]
    data = np.array([[float(row[key]) for key in GT_FIELDS[:-1]] for row in rows])
    trajectory = Trajectory(data[:, 0], data[:, 1:4], data[:, 4:8])
    order = np.argsort(trajectory.time)
    trajectory = Trajectory(trajectory.time[order], trajectory.position[order],
                            trajectory.quaternion[order])
    unique = np.r_[True, np.diff(trajectory.time) > 1e-9]
    trajectory = Trajectory(trajectory.time[unique], trajectory.position[unique],
                            trajectory.quaternion[unique])
    trajectory.validate()
    return trajectory


def interpolate(trajectory: Trajectory, times: np.ndarray,
                max_bracket_s: float = 0.2) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    right = np.searchsorted(trajectory.time, times, side="left")
    clipped_right = np.minimum(len(trajectory.time) - 1, right)
    exact = ((right < len(trajectory.time)) &
             np.isclose(trajectory.time[clipped_right], times,
                        rtol=0.0, atol=1e-10))
    valid = exact | ((right > 0) & (right < len(trajectory.time)))
    left = np.maximum(0, right - 1)
    bracket = trajectory.time[clipped_right] - trajectory.time[left]
    valid &= exact | (bracket <= max_bracket_s)
    query = times[valid]
    positions = np.empty((len(query), 3))
    if not len(query):
        return valid, positions, np.empty((0, 4))
    for axis in range(3):
        positions[:, axis] = np.interp(query, trajectory.time,
                                      trajectory.position[:, axis])
    rotations = Slerp(trajectory.time, Rotation.from_quat(trajectory.quaternion))(query)
    return valid, positions, rotations.as_quat()


def rotation_angle_deg(q_error: Rotation) -> np.ndarray:
    return np.rad2deg(q_error.magnitude())


def stats(values: np.ndarray, prefix: str) -> dict:
    values = np.asarray(values, dtype=float)
    return {f"{prefix}_rmse": float(np.sqrt(np.mean(values ** 2))),
            f"{prefix}_mean": float(np.mean(values)),
            f"{prefix}_median": float(np.median(values)),
            f"{prefix}_p95": float(np.percentile(values, 95)),
            f"{prefix}_max": float(np.max(values))}


def rigid_align(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_mean, target_mean = source.mean(0), target.mean(0)
    covariance = (source - source_mean).T @ (target - target_mean)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation


def evaluate_trajectories(estimate: Trajectory, gt: Trajectory,
                          rate_hz: float = 10.0, delta_s: float = 1.0,
                          max_bracket_s: float = 0.2,
                          sensor_interval: tuple[float, float] | None = None) -> tuple[dict, dict]:
    start = max(estimate.time[0], gt.time[0])
    stop = min(estimate.time[-1], gt.time[-1])
    if stop - start < delta_s:
        raise ValueError("common trajectory interval is too short")
    grid = np.arange(math.ceil(start * rate_hz) / rate_hz,
                     stop + 0.5 / rate_hz, 1.0 / rate_hz)
    ev, ep, eq = interpolate(estimate, grid, max_bracket_s)
    gv, gp_all, gq_all = interpolate(gt, grid, max_bracket_s)
    valid = ev & gv
    # interpolate() returns compact arrays, so repeat on the common grid.
    common = grid[valid]
    _, ep, eq = interpolate(estimate, common, max_bracket_s)
    _, gp, gq = interpolate(gt, common, max_bracket_s)
    if len(common) < 20:
        raise ValueError("fewer than 20 common evaluation samples")
    ape = np.linalg.norm(ep - gp, axis=1)
    qerr = Rotation.from_quat(gq).inv() * Rotation.from_quat(eq)
    metrics = {"status": "SUCCESS", "samples": int(len(common)),
               "grid_samples": int(len(grid)), "coverage": float(len(common) / len(grid)),
               "start_s": float(common[0]), "stop_s": float(common[-1])}
    metrics.update(stats(ape, "ape_translation_m"))
    metrics.update(stats(rotation_angle_deg(qerr), "ape_rotation_deg"))
    er = Rotation.from_quat(eq)
    gr = Rotation.from_quat(gq)
    # GT-fitted fixed rotations are diagnostics only. Check both sides:
    # R_est = Q_world R_gt and R_est = R_gt Q_body.
    for side, errors in (("world_left", er * gr.inv()),
                         ("body_right", gr.inv() * er)):
        fixed = errors.mean()
        metrics[f"diagnostic_{side}_rotation_xyzw"] = fixed.as_quat().tolist()
        metrics.update(stats(rotation_angle_deg(fixed.inv() * errors),
                             f"diagnostic_{side}_residual_deg"))
    up = np.array([0.0, 0.0, 1.0])
    metrics.update(stats(np.rad2deg(np.arccos(np.clip(np.sum(
        er.inv().apply(up) * gr.inv().apply(up), axis=1), -1, 1))),
        "gravity_direction_error_deg"))
    if sensor_interval is not None:
        sensor_start, sensor_stop = sensor_interval
        if not np.isfinite(sensor_interval).all() or sensor_stop <= sensor_start:
            raise ValueError("invalid fixed sensor interval")
        fixed_grid = np.arange(math.ceil(sensor_start * rate_hz) / rate_hz,
                               sensor_stop + 0.5 / rate_hz, 1.0 / rate_hz)
        output_valid, _, _ = interpolate(estimate, fixed_grid, max_bracket_s)
        truth_valid, _, _ = interpolate(gt, fixed_grid, max_bracket_s)
        padded = np.r_[False, ~output_valid, False].astype(int)
        edges = np.diff(padded)
        gaps = np.flatnonzero(edges == -1) - np.flatnonzero(edges == 1)
        metrics.update(sensor_start_s=float(sensor_start), sensor_stop_s=float(sensor_stop),
                       fixed_grid_samples=int(len(fixed_grid)),
                       fixed_paired_samples=int((output_valid & truth_valid).sum()),
                       fixed_interval_coverage=float((output_valid & truth_valid).mean()),
                       valid_output_ratio=float(output_valid.mean()),
                       initialization_duration_s=float(max(0, estimate.time[0]-sensor_start)),
                       no_output_duration_s=float((~output_valid).sum()/rate_hz),
                       max_no_output_gap_s=float(gaps.max()/rate_hz) if len(gaps) else 0.0)
    rot, trans = rigid_align(ep, gp)
    aligned = (rot @ ep.T).T + trans
    metrics.update(stats(np.linalg.norm(aligned - gp, axis=1),
                         "ape_se3_aligned_translation_m"))
    step = int(round(delta_s * rate_hz))
    if len(common) > step:
        contiguous = np.isclose(common[step:] - common[:-step], delta_s,
                                atol=0.51 / rate_hz)
        er = Rotation.from_quat(eq)
        gr = Rotation.from_quat(gq)
        est_delta = er[:-step].inv().apply(ep[step:] - ep[:-step])
        gt_delta = gr[:-step].inv().apply(gp[step:] - gp[:-step])
        rpe_t = np.linalg.norm(est_delta - gt_delta, axis=1)[contiguous]
        est_rel = er[:-step].inv() * er[step:]
        gt_rel = gr[:-step].inv() * gr[step:]
        rpe_r = rotation_angle_deg(gt_rel.inv() * est_rel)[contiguous]
        world_delta = np.linalg.norm((ep[step:]-ep[:-step]) -
                                    (gp[step:]-gp[:-step]), axis=1)[contiguous]
        # Gaps in either trajectory can leave no valid one-second pairs even
        # when the common-position coverage is otherwise adequate.  Keep the
        # run valid and report RPE as unavailable instead of emitting NaNs.
        if len(rpe_t):
            metrics.update(stats(rpe_t, "rpe_1s_translation_m"))
            metrics.update(stats(rpe_r, "rpe_1s_rotation_deg"))
            metrics.update(stats(world_delta, "world_position_increment_1s_m"))
    samples = {"time": common.tolist(), "ape_translation_m": ape.tolist(),
               "estimate": ep.tolist(), "ground_truth": gp.tolist()}
    if len(common) > step:
        samples["rpe_time"] = common[step:][contiguous].tolist()
        samples["rpe_1s_translation_m"] = rpe_t.tolist()
        samples["rpe_1s_rotation_deg"] = rpe_r.tolist()
        samples["world_position_increment_1s_m"] = world_delta.tolist()
    return metrics, samples


def percentile(values: Sequence[float], q: float) -> float | None:
    return float(np.percentile(values, q)) if values else None


def validate_cache(cache: Path) -> dict:
    manifest = json.loads((cache / "manifest.json").read_text())
    errors = []
    if manifest.get("schema") != CACHE_SCHEMA:
        errors.append("SCHEMA")
    for name in ("imu.csv", "uwb.csv", "anchors.csv"):
        path = cache / name
        if not path.is_file():
            errors.append("MISSING_" + name.upper())
        elif manifest.get("files", {}).get(name, {}).get("sha256") != sha256(path):
            errors.append("HASH_" + name.upper())
    imu = read_csv(cache / "imu.csv") if (cache / "imu.csv").exists() else []
    uwb = read_csv(cache / "uwb.csv") if (cache / "uwb.csv").exists() else []
    anchors = read_csv(cache / "anchors.csv") if (cache / "anchors.csv").exists() else []
    gt = read_csv(cache / "gt.csv") if (cache / "gt.csv").exists() else []
    expected_headers = {"imu.csv": IMU_FIELDS, "uwb.csv": UWB_FIELDS,
                        "anchors.csv": ANCHOR_FIELDS, "gt.csv": GT_FIELDS}
    for name, expected in expected_headers.items():
        path = cache/name
        if path.is_file():
            with path.open(newline="") as stream:
                actual = tuple(next(csv.reader(stream)))
            if actual != tuple(expected):
                errors.append("FIELDS_" + name.upper())
    for name, rows in (("IMU", imu), ("UWB", uwb)):
        times = np.array([float(row["t"]) for row in rows]) if rows else np.array([])
        if len(times) == 0 or not np.isfinite(times).all():
            errors.append(name + "_EMPTY_OR_NONFINITE")
        elif (name == "IMU" and np.any(np.diff(times) <= 0)) or \
                (name == "UWB" and np.any(np.diff(times) < -1e-9)):
            errors.append(name + "_NONMONOTONIC")
        try:
            numeric = np.asarray([[float(value) for value in row.values()]
                                  for row in rows])
            if not np.isfinite(numeric).all():
                errors.append(name + "_NONFINITE")
        except (TypeError, ValueError):
            errors.append(name + "_NONNUMERIC")
    anchor_ids = {int(row["anchor_id"]) for row in anchors}
    observed_ids = {int(row["anchor_id"]) for row in uwb}
    if len(anchor_ids) < 4 or not observed_ids.issubset(anchor_ids):
        errors.append("ANCHOR_IDS")
    pairs = [(int(row["source_message"]), int(row["source_range"])) for row in uwb]
    if len(pairs) != len(set(pairs)):
        errors.append("DUPLICATE_SOURCE_RANGE")
    if manifest.get("primary_tag") not in {int(row["tag_id"]) for row in uwb}:
        errors.append("PRIMARY_TAG")
    if "gt.csv" in manifest.get("estimator_inputs", []) or \
            "gt.csv" not in manifest.get("evaluator_only", []):
        errors.append("GT_INPUT_ISOLATION")
    matrix = manifest.get("provenance", {}).get("rotation_body_from_imu_matrix")
    transform_orthogonality = None
    if matrix is not None:
        matrix = np.asarray(matrix, float)
        transform_orthogonality = float(np.linalg.norm(matrix.T@matrix-np.eye(3)))
        if matrix.shape != (3, 3) or transform_orthogonality > 1e-6 or \
                abs(np.linalg.det(matrix)-1.0) > 1e-6:
            errors.append("IMU_ROTATION_NOT_SO3")
    gravity = [np.linalg.norm([float(r["ax"]), float(r["ay"]), float(r["az"])])
               for r in imu[:min(1000, len(imu))]]
    valid_uwb = sum(int(row["valid"]) for row in uwb)
    imu_times = np.asarray([float(row["t"]) for row in imu])
    imu_dt = np.diff(imu_times)
    imu_frequency = float(1.0/np.median(imu_dt)) if len(imu_dt) else None
    gt_coverage = 0.0
    geometry_median = geometry_p95 = None
    if gt and uwb:
        primary = int(manifest["primary_tag"])
        gt_rows = [row for row in gt if int(row["tag_id"]) == primary]
        range_rows = [row for row in uwb if int(row["tag_id"]) == primary and
                      int(row["valid"]) != 0]
        if gt_rows and range_rows:
            gt_t = np.asarray([float(row["t"]) for row in gt_rows])
            gt_position = np.asarray([[float(row[key]) for key in
                                       ("px", "py", "pz")] for row in gt_rows])
            inside = [row for row in range_rows
                      if gt_t[0] <= float(row["t"]) <= gt_t[-1]]
            gt_coverage = len(inside)/len(range_rows)
            sampled = inside[::max(1, len(inside)//2000)]
            residual = []
            for row in sampled:
                t = float(row["t"])
                tag = np.array([np.interp(t, gt_t, gt_position[:, axis])
                                for axis in range(3)])
                anchor = np.array([float(row[key]) for key in
                                   ("anchor_x", "anchor_y", "anchor_z")])
                residual.append(abs(np.linalg.norm(tag-anchor)-float(row["range"])))
            if residual:
                geometry_median = float(np.median(residual))
                geometry_p95 = float(np.percentile(residual, 95))
    return {"status": "PASS" if not errors else "FAIL", "errors": errors,
            "imu_count": len(imu), "uwb_count": len(uwb),
            "valid_uwb_count": valid_uwb,
            "imu_frequency_hz": imu_frequency,
            "imu_max_gap_s": float(np.max(imu_dt)) if len(imu_dt) else None,
            "initial_accel_norm_median": float(np.median(gravity)) if gravity else None,
            "gt_time_coverage": gt_coverage,
            "range_geometry_abs_residual_median": geometry_median,
            "range_geometry_abs_residual_p95": geometry_p95,
            "transform_orthogonality_error": transform_orthogonality,
            "manifest": str(cache / "manifest.json")}
