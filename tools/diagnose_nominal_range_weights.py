#!/usr/bin/env python3
"""Reconstruct posterior range residuals from frozen runs, without reading GT.

States are rounded CSV exports. These are posterior-implied scalar Huber
weights, not the solver's historical IRLS weights or a full navigation Hessian.
Batch formation matches nominal_dataset_runner; exact diagnostic timestamps and
range counts bind every row before residuals are accepted.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def identity(path):
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def distribution(values):
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    return {"count": int(values.size), "nonfinite": int(values.size - finite.size),
            **({"mean": float(np.mean(finite)),
                "p50": float(np.quantile(finite, .5)),
                "p95": float(np.quantile(finite, .95)),
                "p99": float(np.quantile(finite, .99)),
                "max": float(np.max(finite))} if finite.size else {})}


def batches(cache, manifest):
    ranges = [r for r in rows(cache / "uwb.csv")
              if int(r["valid"]) and float(r["range"]) > 0 and float(r["sigma"]) > 0]
    window = manifest.get("epoch_batch_window_s", 0)
    output = {}
    begin = 0
    while begin < len(ranges):
        end = begin + 1
        while end < len(ranges) and (
                ranges[end]["source_message"] == ranges[begin]["source_message"] or
                float(ranges[end]["t"]) - float(ranges[begin]["t"]) <= window + 1e-12):
            end += 1
        stamp = round(float(ranges[end - 1]["t"]) * 1e9)
        selected = [r for r in ranges[begin:end]
                    if int(r["tag_id"]) == manifest["primary_tag"]]
        if selected:
            if stamp in output:
                raise ValueError("duplicate batch timestamp")
            output[stamp] = selected
        begin = end
    return output


def rotation(state):
    q = np.array([float(state[k]) for k in ("qx", "qy", "qz", "qw")])
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])


def analyze(run, native_batches, robust, manifest_hash):
    states, diagnostics = rows(run / "states.csv"), rows(run / "sensor_diagnostics.csv")
    if len(states) != len(diagnostics):
        raise ValueError("state/diagnostic row count mismatch")
    execution = json.loads((run / "execution.json").read_text())
    command = execution["command"]
    if "--tag-policy" in command and command[command.index("--tag-policy")+1] != "primary":
        raise ValueError("this diagnostic binds primary-tag runs only")
    if execution["exit_code"] != 0:
        raise ValueError("source run did not complete")
    config_path = run / "effective_nominal_config.yaml"
    if identity(config_path)["sha256"] != execution["effective_config_sha256"]:
        raise ValueError("effective config changed after source run")
    if manifest_hash != execution["cache_manifest_sha256"]:
        raise ValueError("cache manifest changed after source run")
    config = yaml.safe_load(config_path.read_text())
    if config.get("estimation_tuning", {}).get("nominal_robust_experimental", False) != robust:
        raise ValueError("source config does not match requested Gaussian/Huber role")
    z_values, weights, loss, information_ratios, covariance_checks = [], [], [], [], []
    raw_conditions, weighted_conditions = [], []
    by_anchor = {}
    signatures = []
    for state, diagnostic in zip(states, diagnostics):
        stamp = round(float(diagnostic["t"]) * 1e9)
        # Runner states use six significant digits; diagnostic t has 17.
        if abs(float(state["t"]) - float(diagnostic["t"])) > 6e-6 * max(1, abs(float(diagnostic["t"]))):
            raise ValueError("rounded state timestamp does not bind diagnostic")
        batch = native_batches[stamp]
        if len(batch) != int(diagnostic["range_count"]):
            raise ValueError("native batch range count does not bind diagnostic")
        signatures.append([stamp, len(batch)])
        position = np.array([float(state[k]) for k in ("px", "py", "pz")])
        rot = rotation(state)
        z, los = [], []
        for obs in batch:
            lever = np.array([float(obs[f"lever_{a}"]) for a in "xyz"])
            anchor = np.array([float(obs[f"anchor_{a}"]) for a in "xyz"])
            delta = position + rot @ lever - anchor
            predicted = np.linalg.norm(delta)
            if predicted < 1e-9:
                raise ValueError("range Jacobian singular at anchor")
            residual = (predicted - float(obs["range"])) / float(obs["sigma"])
            z.append(residual)
            los.append(delta / (predicted * float(obs["sigma"])))
            by_anchor.setdefault(obs["anchor_id"], []).append(residual)
        z, los = np.asarray(z), np.asarray(los)
        implied = np.minimum(1., 1.5 / np.maximum(np.abs(z), np.finfo(float).tiny))
        applied = implied if robust else np.ones_like(implied)
        raw = los.T @ los
        weighted = los.T @ (applied[:, None] * los)
        raw_eigen = np.linalg.eigvalsh(raw)
        weighted_eigen = np.linalg.eigvalsh(weighted)
        if raw_eigen[0] > 1e-12 * raw_eigen[-1]:
            information_ratios.append(weighted_eigen[0] / raw_eigen[0])
            raw_conditions.append(raw_eigen[-1] / raw_eigen[0])
        else:
            raw_conditions.append(float("inf"))
        weighted_conditions.append(weighted_eigen[-1] / weighted_eigen[0]
                                   if weighted_eigen[0] > 1e-12 * weighted_eigen[-1]
                                   else float("inf"))
        loss.append(float(np.trace(weighted) / np.trace(raw)))
        expected = float(diagnostic["posterior_whitened_sq"])
        deviation = abs(float(z @ z) - expected)
        covariance_checks.append(deviation / max(1., expected))
        # Fail closed if rounded states cannot reproduce the logged Gaussian
        # cost. This tolerance is for lossy CSV reconstruction, not PL numerics.
        if deviation > .005 * max(1., expected):
            raise ValueError("posterior residual reconstruction does not match logged cost")
        z_values.extend(z.tolist())
        weights.extend(implied.tolist())
    z, weights = np.array(z_values), np.array(weights)
    return {"rows": len(states), "batch_signature_sha256": hashlib.sha256(
                json.dumps(signatures).encode()).hexdigest(),
            "source_files": [identity(run / f) for f in (
                "states.csv", "sensor_diagnostics.csv", "execution.json", "effective_nominal_config.yaml")],
            "standardized_residual_abs": distribution(np.abs(z)),
            "signed_residual": distribution(z),
            "implied_huber_weight": distribution(weights),
            "fraction_above_huber_k": float(np.mean(weights < 1.)),
            "fraction_weight_below_half": float(np.mean(weights < .5)),
            "conditional_position_information_trace_ratio": distribution(loss),
            "conditional_position_information_min_eigen_ratio": distribution(information_ratios),
            "conditional_position_information_condition_unweighted": distribution(raw_conditions),
            "conditional_position_information_condition_applied": distribution(weighted_conditions),
            "rounded_export_cost_relative_deviation": distribution(covariance_checks),
            "per_anchor_signed_standardized_residual": {
                k: distribution(v) for k, v in sorted(by_anchor.items())}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("cache", "gaussian", "robust", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.cache / "manifest.json").read_text())
    native = batches(args.cache, manifest)
    manifest_hash = identity(args.cache / "manifest.json")["sha256"]
    g = analyze(args.gaussian, native, False, manifest_hash)
    h = analyze(args.robust, native, True, manifest_hash)
    if g["batch_signature_sha256"] != h["batch_signature_sha256"]:
        raise ValueError("paired output batch coverage differs")
    result = {"schema": "uwb-imu-pl/posterior-range-weight-diagnostic/v1",
              "gt_read": False, "parameter_selection_performed": False,
              "cache_identity": [identity(args.cache / f) for f in ("manifest.json", "uwb.csv")],
              "calibration_label": manifest.get("calibration_label"),
              "frame_provenance": manifest.get("provenance"),
              "huber_k": 1.5, "gaussian": g, "robust": h,
              "limitations": ["Rounded posterior exports; implied weights are not historical solver IRLS weights.",
                              "Position information conditions on fixed attitude; excludes IMU/prior/history and is not navigation covariance.",
                              "Residuals cannot alone identify NLOS, noise calibration, extrinsic or attitude error."]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
