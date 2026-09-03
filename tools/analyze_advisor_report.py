#!/usr/bin/env python3
"""Validate report raw data and generate summaries, figures, and LaTeX macros."""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import pathlib
import shutil
import sys
from collections import defaultdict
from typing import Any, Iterable

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import numpy as np
import pandas as pd
from scipy.stats import beta, binomtest, chi2

from advisor_report_common import atomic_json, load_protocol, verify_checksums

BLUE = "#0072B2"
ORANGE = "#E69F00"
RED = "#D55E00"
GREEN = "#009E73"
BLACK = "#202020"
GRAY = "#777777"


def percentile(values: np.ndarray, q: float) -> float:
    return float(np.quantile(values[np.isfinite(values)], q))


def read(path: pathlib.Path) -> pd.DataFrame | None:
    if not path.is_file():
        return None
    frame = pd.read_csv(path)
    if frame.empty:
        raise ValueError(f"empty raw file: {path}")
    return frame


def validate(frame: pd.DataFrame, name: str,
             expected_range: tuple[int, int] | None = None) -> dict[str, Any]:
    keys = (["source_run"] if "source_run" in frame else []) + [
        "seed", "trajectory", "graph_history", "epoch"]
    if frame.duplicated(keys).any():
        raise ValueError(f"{name}: duplicate mode epoch")
    required = ["position_error_m", "velocity_error_mps", "orientation_error_rad",
                "cov_xx", "cov_yy", "cov_zz", "position_nees", "conditional_statistic",
                "conditional_threshold", "horizontal_error_m", "vertical_error_m",
                "core_ms", "active_values", "active_factors"]
    values = frame[required].apply(pd.to_numeric, errors="coerce").to_numpy()
    if not np.isfinite(values).all():
        raise ValueError(f"{name}: non-finite required value")
    digest_counts = frame.groupby(["seed", "trajectory"])["input_digest"].nunique()
    if not (digest_counts == 1).all():
        raise ValueError(f"{name}: paired modes do not share one input digest")
    sequence_keys = [field for field in keys if field != "epoch"]
    sequence_sizes = frame.groupby(sequence_keys).size()
    if sequence_sizes.nunique() != 1:
        raise ValueError(f"{name}: incomplete sequence")
    epochs = int(sequence_sizes.iloc[0])
    first_epoch = int(frame.epoch.min())
    last_epoch = int(frame.epoch.max())
    expected = set(range(first_epoch, last_epoch + 1))
    if expected_range is not None and (first_epoch, last_epoch) != expected_range:
        raise ValueError(f"{name}: output range {(first_epoch, last_epoch)} != {expected_range}")
    for block, rows in frame.groupby(sequence_keys):
        if set(rows["epoch"].astype(int)) != expected:
            raise ValueError(f"{name}: non-contiguous epochs in {block}")
    finite_columns = frame.hpl_m.notna() & frame.vpl_m.notna()
    if not np.array_equal(finite_columns.to_numpy(), frame.finite_pl.astype(bool).to_numpy()):
        raise ValueError(f"{name}: finite_pl disagrees with HPL/VPL columns")
    available = frame.availability == "AVAILABLE"
    if (available & (~frame.finite_pl.astype(bool) |
                     ~frame.conditional_passed.astype(bool) |
                     ~frame.measurement_model_valid.astype(bool))).any():
        raise ValueError(f"{name}: AVAILABLE without valid detector and finite PL")
    # Model/risk invalidity has priority and is UNAVAILABLE.  ALERT is required
    # for a numerically valid rejection only when the PL model/risk gate is
    # itself formally eligible.
    detector_reject = (~frame.conditional_passed.astype(bool) &
                       frame.formal_eligible.astype(bool))
    if (detector_reject & ((frame.availability != "ALERT") |
                          frame.finite_pl.astype(bool) |
                          frame.batch_committed.astype(bool))).any():
        raise ValueError(f"{name}: detector rejection did not fail closed")
    model_invalid = ~frame.measurement_model_valid.astype(bool)
    if (model_invalid & (frame.availability != "UNAVAILABLE")).any():
        raise ValueError(f"{name}: invalid model did not report UNAVAILABLE")
    return {"rows": len(frame), "sequences": len(sequence_sizes), "epochs": epochs,
            "output_epoch_range": [first_epoch, last_epoch],
            "histories": sorted(map(int, frame.graph_history.unique())),
            "paired_input_digest": True, "state_invariants": True}


def clopper_pearson_upper(successes: int, trials: int,
                          confidence: float = .95) -> float:
    """One-sided exact binomial upper confidence bound."""
    if trials < 0 or successes < 0 or successes > trials:
        raise ValueError("invalid binomial counts")
    if trials == 0 or successes == trials:
        return 1.0
    return float(beta.ppf(confidence, successes + 1, trials - successes))


def _pl_group(rows: pd.DataFrame, confidence: float) -> dict[str, Any]:
    eligible = ((rows.availability == "AVAILABLE") &
                rows.conditional_passed.astype(bool) & rows.finite_pl.astype(bool))
    horizontal_ok = rows.horizontal_error_m <= rows.hpl_m
    vertical_ok = rows.vertical_error_m <= rows.vpl_m
    contained = eligible & horizontal_ok & vertical_ok
    hmi = eligible & (~horizontal_ok | ~vertical_ok)
    total = len(rows)
    denominator = int(eligible.sum())
    hmi_count = int(hmi.sum())
    return {
        "samples": total,
        "finite_pl_denominator": denominator,
        "contained": int(contained.sum()),
        "containment_rate": float(contained.sum() / denominator)
            if denominator else None,
        "reject_rate": float((~rows.conditional_passed.astype(bool)).mean()),
        "alert_rate": float((rows.availability == "ALERT").mean()),
        "unavailable_rate": float((rows.availability == "UNAVAILABLE").mean()),
        "hmi_count": hmi_count,
        "hmi_rate": float(hmi_count / total) if total else None,
        "hmi_cp95_upper": clopper_pearson_upper(hmi_count, total, confidence),
    }


def native_scalar(value: Any) -> Any:
    return value.item() if isinstance(value, np.generic) else value


def isolated_pl_summary(frame: pd.DataFrame, protocol: dict[str, Any]) -> tuple[dict[str, Any], pd.DataFrame]:
    spec = protocol["isolated_pl"]
    active = frame[frame.fault_active.astype(bool)].copy()
    sequence_keys = ["seed", "trajectory", "graph_history"]
    if (active.groupby(sequence_keys).size() != 1).any():
        raise ValueError("isolated_pl: each mode sequence must contain one fault epoch")
    active["statistic_ratio"] = active.conditional_statistic / active.conditional_threshold
    confidence = float(spec["clopper_pearson_confidence"])
    group_fields = ["graph_history", "trajectory", "anchor_id", "fault_m"]
    rows = []
    for key, group in active.groupby(group_fields, sort=True):
        item = {field: native_scalar(value) for field, value in zip(group_fields, key)}
        item.update(_pl_group(group, confidence))
        rows.append(item)
    table = pd.DataFrame(rows)
    expected = int(spec["fault"]["seeds_per_anchor_magnitude_cell"])
    if len(active.seed.unique()) == int(spec["seeds"]["count"]):
        if table.empty or not (table.samples == expected).all():
            raise ValueError("isolated_pl: unbalanced anchor-magnitude cells")
    low, high = map(float, spec["near_boundary_ratio"])
    near = active[active.statistic_ratio.between(low, high, inclusive="both")]
    result = {
        "fault_epoch_samples": len(active),
        "unique_seeds": int(active.seed.nunique()),
        "overall": _pl_group(active, confidence),
        "near_boundary": {"ratio_range": [low, high], "samples": len(near),
                          **_pl_group(near, confidence)},
        "cells": rows,
    }
    return result, table


def longest_true_run(values: Iterable[bool]) -> int:
    longest = current = 0
    for value in values:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest


def persistent_fault_summary(frame: pd.DataFrame) -> tuple[dict[str, Any], pd.DataFrame]:
    active = frame[frame.fault_active.astype(bool)].copy()
    sequence_fields = ["graph_history", "trajectory", "anchor_id", "seed"]
    sequence_rows = []
    for key, rows in active.groupby(sequence_fields, sort=True):
        rows = rows.sort_values("epoch")
        rejected = ~rows.batch_committed.astype(bool)
        item = {field: native_scalar(value) for field, value in zip(sequence_fields, key)}
        item.update({
            "fault_epochs": len(rows),
            "detection_rate": float((~rows.conditional_passed.astype(bool)).mean()),
            "ever_detected": bool((~rows.conditional_passed.astype(bool)).any()),
            "longest_rejection_epochs": longest_true_run(rejected),
            "alert_rate": float((rows.availability == "ALERT").mean()),
            "unavailable_rate": float((rows.availability == "UNAVAILABLE").mean()),
            "max_imu_only_drift_m": float(rows.loc[rejected, "position_error_m"].max())
                if rejected.any() else 0.0,
            "rejected_with_finite_pl_violations": int((rejected & rows.finite_pl.astype(bool)).sum()),
        })
        sequence_rows.append(item)
    table = pd.DataFrame(sequence_rows)
    rejected = ~active.batch_committed.astype(bool)
    result = {
        "fault_epoch_rows": len(active),
        "sequences": len(table),
        "detection_rate": float((~active.conditional_passed.astype(bool)).mean()),
        "ever_detected_sequence_rate": float(table.ever_detected.mean()) if len(table) else None,
        "maximum_continuous_rejection_epochs": int(table.longest_rejection_epochs.max()) if len(table) else 0,
        "alert_rate": float((active.availability == "ALERT").mean()),
        "unavailable_rate": float((active.availability == "UNAVAILABLE").mean()),
        "maximum_imu_only_drift_m": float(active.loc[rejected, "position_error_m"].max())
            if rejected.any() else 0.0,
        "rejected_with_finite_pl_violations": int((rejected & active.finite_pl.astype(bool)).sum()),
    }
    return result, table


def representative_trajectory_rows(frame: pd.DataFrame) -> pd.DataFrame:
    rows = frame[frame.trajectory == "figure_eight"].copy()
    if rows.empty or rows.trajectory.nunique() != 1:
        raise ValueError("representative trajectory must contain figure_eight only")
    rows = rows.sort_values(["graph_history", "epoch"], kind="mergesort")
    for _, mode in rows.groupby("graph_history"):
        if not mode.epoch.is_monotonic_increasing or mode.epoch.duplicated().any():
            raise ValueError("representative trajectory epochs are not strictly ordered")
    return rows


def block_bootstrap(values: np.ndarray, trials: int, seed: int) -> list[float]:
    rng = np.random.default_rng(seed)
    if not len(values):
        return [math.nan, math.nan]
    samples = rng.choice(values, size=(trials, len(values)), replace=True).mean(axis=1)
    return [float(np.quantile(samples, .025)), float(np.quantile(samples, .975))]


def holm(p_values: list[float], alpha: float) -> list[dict[str, Any]]:
    order = sorted(range(len(p_values)), key=lambda index: (p_values[index], index))
    output: list[dict[str, Any] | None] = [None] * len(p_values)
    still_rejecting = True
    for rank, index in enumerate(order, 1):
        threshold = alpha / (len(p_values)-rank+1)
        rejected = still_rejecting and p_values[index] <= threshold
        if not rejected:
            still_rejecting = False
        output[index] = {"p_value": p_values[index], "rank": rank,
                         "threshold": threshold, "rejected": rejected}
    return [item for item in output if item is not None]


def nominal_summary(frame: pd.DataFrame, warmup: int, trials: int,
                    seed: int) -> dict[str, Any]:
    warmup = min(warmup, max(0, int(frame.epoch.max()) // 5))
    used = frame[frame.epoch >= warmup].copy()
    result: dict[str, Any] = {"warmup_epochs": warmup, "modes": {}}
    block_rmse: dict[int, dict[tuple[int, str], float]] = defaultdict(dict)
    for history, rows in used.groupby("graph_history"):
        history = int(history)
        error = rows.position_error_m.to_numpy(float)
        horizontal = np.hypot(rows.est_px-rows.truth_px, rows.est_py-rows.truth_py)
        vertical = np.abs(rows.est_pz-rows.truth_pz)
        axis = np.column_stack((rows.est_px-rows.truth_px, rows.est_py-rows.truth_py,
                                rows.est_pz-rows.truth_pz))
        for block, block_rows in rows.groupby(["seed", "trajectory"]):
            block_rmse[history][block] = float(np.sqrt(np.mean(
                np.square(block_rows.position_error_m))))
        result["modes"][str(history)] = {
            "ate_rmse_m": float(np.sqrt(np.mean(error**2))),
            "position_error_p50_m": percentile(error, .50),
            "position_error_p95_m": percentile(error, .95),
            "position_error_p99_m": percentile(error, .99),
            "axis_rmse_m": np.sqrt(np.mean(axis**2, axis=0)).tolist(),
            "velocity_rmse_mps": float(np.sqrt(np.mean(rows.velocity_error_mps**2))),
            "orientation_rmse_rad": float(np.sqrt(np.mean(rows.orientation_error_rad**2))),
            "position_nees_mean": float(rows.position_nees.mean()),
            "position_covariance_coverage_95": float(np.mean(rows.position_nees <= chi2.ppf(.95, 3))),
            "hpl_containment": float(np.mean(horizontal <= rows.hpl_m)),
            "vpl_containment": float(np.mean(vertical <= rows.vpl_m)),
            "availability": float(np.mean(rows.availability == "AVAILABLE")),
            "false_rejection": float(np.mean(rows.batch_committed == 0)),
            "observed_hmi": int(np.sum((horizontal > rows.hpl_m) | (vertical > rows.vpl_m))),
            "core_mean_ms": float(rows.core_ms.mean()),
            "core_p95_ms": percentile(rows.core_ms.to_numpy(float), .95),
            "core_p99_ms": percentile(rows.core_ms.to_numpy(float), .99),
            "active_values_max": int(rows.active_values.max()),
            "active_factors_max": int(rows.active_factors.max()),
            "peak_rss_mb": float(rows.peak_rss_mb.max()),
        }
    paired = {"status": "NOT_AVAILABLE"}
    if 0 in block_rmse and 200 in block_rmse:
        common = sorted(set(block_rmse[0]) & set(block_rmse[200]))
        relative = np.asarray([block_rmse[200][key] / block_rmse[0][key] - 1
                               for key in common])
        ci = block_bootstrap(relative, trials, seed)
        paired = {"status": "AVAILABLE", "blocks": len(relative),
                  "mean_relative_ate_difference": float(relative.mean()),
                  "ci95": ci, "criterion_ci_upper_le_5pct": ci[1] <= .05}
    result["paired_fixed_lag_vs_full_history"] = paired
    return result


def auc(nominal: np.ndarray, fault: np.ndarray) -> float:
    combined = np.concatenate((nominal, fault))
    order = np.argsort(combined, kind="mergesort")
    ranks = np.empty(len(combined), float)
    ranks[order] = np.arange(1, len(combined)+1)
    _, inverse, counts = np.unique(combined, return_inverse=True, return_counts=True)
    sums = np.bincount(inverse, weights=ranks)
    ranks = sums[inverse] / counts[inverse]
    n0, n1 = len(nominal), len(fault)
    return float((ranks[n0:].sum() - n1*(n1+1)/2) / (n0*n1))


def roc_summary(calibration: pd.DataFrame, nominal: pd.DataFrame,
                fault: pd.DataFrame, protocol: dict[str, Any], profile: str,
                trials: int, seed: int) -> dict[str, Any]:
    detectors = {"global": "global_statistic", "postfit": "postfit_statistic",
                 "conditional": "conditional_statistic"}
    result: dict[str, Any] = {"detectors": {}, "operating_points": []}
    pfa_p_values: list[float] = []
    thresholds: dict[tuple[int, str, float], float] = {}
    for history in sorted(map(int, calibration.graph_history.unique())):
        result["detectors"][str(history)] = {}
        for detector, field in detectors.items():
            cal = calibration[calibration.graph_history == history][field].dropna().to_numpy(float)
            nom = nominal[nominal.graph_history == history][field].dropna().to_numpy(float)
            bad = fault[(fault.graph_history == history) & (fault.fault_active == 1)][field].dropna().to_numpy(float)
            if not len(cal) or not len(nom) or not len(bad):
                continue
            result["detectors"][str(history)][detector] = {"auc": auc(nom, bad)}
            for pfa in protocol["roc"]["operating_p_fa"]:
                threshold = float(np.quantile(cal, 1-float(pfa), method="higher"))
                thresholds[(history, detector, float(pfa))] = threshold
                false_alarms = int(np.sum(nom > threshold))
                detections = int(np.sum(bad > threshold))
                pfa_p_values.append(float(binomtest(false_alarms, len(nom), float(pfa)).pvalue))
                result["operating_points"].append({
                    "history": history, "detector": detector, "target_p_fa": float(pfa),
                    "threshold": threshold, "nominal_count": len(nom),
                    "false_alarms": false_alarms, "empirical_p_fa": false_alarms/len(nom),
                    "fault_count": len(bad), "detections": detections,
                    "power": detections/len(bad)})
    corrected = holm(pfa_p_values, float(protocol["roc"]["familywise_alpha"]))
    for item, correction in zip(result["operating_points"], corrected):
        item["pfa_holm"] = correction
    result["pfa_holm_pass"] = not any(item["rejected"] for item in corrected)
    theory = float(protocol["roc"]["theory_only_p_fa"])
    result["theory_only"] = {"p_fa": theory, "chi2_dof8_threshold": float(chi2.ppf(1-theory, 8)),
                             "rare_event_certified": False}
    target = float(protocol["roc"]["noninferiority"]["operating_p_fa"])
    min_fault = float(protocol["roc"]["noninferiority"]["minimum_fault_magnitude_m"])
    field = detectors[protocol["roc"]["noninferiority"]["detector"]]
    differences = []
    if (0, "conditional", target) in thresholds and (200, "conditional", target) in thresholds:
        active = fault[(fault.fault_active == 1) & (fault.fault_m >= min_fault)]
        for block, rows in active.groupby(["seed", "trajectory"]):
            powers = {}
            for history in (0, 200):
                item = rows[rows.graph_history == history]
                powers[history] = float(np.mean(item[field] > thresholds[(history, "conditional", target)]))
            if len(powers) == 2:
                differences.append(powers[200]-powers[0])
    if differences:
        values = np.asarray(differences)
        ci = block_bootstrap(values, trials, seed+1)
        result["conditional_noninferiority"] = {
            "blocks": len(values), "mean_power_difference": float(values.mean()),
            "ci95": ci, "margin": float(protocol["roc"]["noninferiority"]["margin"]),
            "pass": ci[0] >= float(protocol["roc"]["noninferiority"]["margin"])}
    else:
        result["conditional_noninferiority"] = {"status": "NOT_AVAILABLE"}
    # TTD at the preregistered conditional point, isolated per seed/trajectory/mode.
    ttd = []
    for history in (0, 200):
        threshold = thresholds.get((history, "conditional", target))
        if threshold is None:
            continue
        for block, rows in fault[fault.graph_history == history].groupby(["seed", "trajectory"]):
            active = rows[rows.fault_active == 1].sort_values("epoch")
            hits = active[active.conditional_statistic > threshold]
            if not active.empty:
                onset = int(active.epoch.iloc[0])
                ttd.append({"history": history, "seed": int(block[0]), "trajectory": block[1],
                            "ttd_epochs": int(hits.epoch.iloc[0])-onset if not hits.empty else None})
    result["time_to_detect"] = ttd
    result["profile"] = profile
    result["formal_gate_claimed"] = profile == "PREREGISTERED_FULL"
    return result


def placeholder(path: pathlib.Path, title: str, detail: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.axis("off")
    ax.text(.5, .58, title, ha="center", va="center", fontsize=18, color=GRAY)
    ax.text(.5, .40, detail, ha="center", va="center", fontsize=11, color=GRAY)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def architecture(path: pathlib.Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 4.4)); ax.axis("off")
    boxes = [("IMU / UWB", .02, BLUE), ("Ordered events", .19, BLUE),
             ("Preintegration", .36, BLUE), ("iSAM2 / fixed-lag", .53, ORANGE),
             ("Integrity + PL", .70, GREEN), ("Commit / reject", .87, RED)]
    for label, x, color in boxes:
        patch = FancyBboxPatch((x, .38), .12, .24, boxstyle="round,pad=.02",
                               facecolor=color, edgecolor="none", alpha=.92)
        ax.add_patch(patch); ax.text(x+.06, .5, label, ha="center", va="center",
                                    color="white", fontsize=10, weight="bold", wrap=True)
    for (_, x, _), (_, nx, _) in zip(boxes, boxes[1:]):
        ax.add_patch(FancyArrowPatch((x+.12, .5), (nx, .5), arrowstyle="-|>",
                                     mutation_scale=16, color=BLACK))
    ax.text(.76, .22, "Detect before current UWB commit", ha="center", color=RED, fontsize=12)
    ax.add_patch(FancyArrowPatch((.91, .36), (.76, .28), connectionstyle="arc3,rad=-.25",
                                 arrowstyle="-|>", color=RED))
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def make_figures(figures: pathlib.Path, frames: dict[str, pd.DataFrame | None],
                 summaries: dict[str, Any], week4: dict[str, Any]) -> None:
    figures.mkdir(parents=True, exist_ok=True)
    architecture(figures/"architecture.pdf")
    representative_trajectory = frames.get("representative_trajectory")
    if representative_trajectory is None:
        placeholder(figures/"trajectories.pdf", "Figure-eight representative trajectory",
                    "Dedicated 1000-epoch sequence not executed")
    else:
        sample = representative_trajectory_rows(representative_trajectory)
        fig, ax = plt.subplots(figsize=(9.5, 4.8))
        truth_rows = sample[sample.graph_history == sample.graph_history.min()].sort_values("epoch")
        ax.plot(truth_rows.truth_px, truth_rows.truth_py, color=BLACK, ls="--",
                lw=1.5, label="GT", zorder=3)
        for history, color, label in ((0, BLUE, "full"), (200, ORANGE, "lag 200")):
            rows = sample[sample.graph_history == history].sort_values("epoch")
            if not rows.empty:
                ax.plot(rows.est_px, rows.est_py, color=color, lw=1.0, label=label)
        ax.set(xlabel="x [m]", ylabel="y [m]", title="1000-epoch figure-eight (top view)")
        ax.axis("equal"); ax.grid(alpha=.25); ax.legend(loc="upper left", fontsize=8)
        anchors = np.asarray([[-5,-5,1],[-5,-5,5],[-5,5,1],[-5,5,5],
                              [5,-5,1],[5,-5,5],[5,5,1],[5,5,5]])
        inset = fig.add_axes([.68, .56, .27, .35], projection="3d")
        inset.scatter(*anchors.T, marker="^", color=GREEN, s=22)
        inset.set(xlabel="x", ylabel="y", zlabel="z", title="8-anchor layout")
        inset.tick_params(labelsize=6); inset.title.set_fontsize(8)
        fig.subplots_adjust(left=.08, right=.96, bottom=.14, top=.90)
        fig.savefig(figures/"trajectories.pdf"); plt.close(fig)
    nominal = frames.get("nominal")
    if nominal is None:
        for name, title in (("nominal_accuracy", "Nominal accuracy"),
                            ("consistency", "Consistency and integrity")):
            placeholder(figures/f"{name}.pdf", title, "Preregistered campaign not executed")
    else:
        modes = summaries["nominal"]["modes"]
        labels = ["Full" if k == "0" else f"Lag {k}" for k in modes]
        values = [modes[k]["ate_rmse_m"] for k in modes]
        fig, ax = plt.subplots(figsize=(7, 4)); ax.bar(labels, values, color=[BLUE if k=="0" else ORANGE for k in modes])
        ax.set_ylabel("ATE RMSE [m]"); ax.grid(axis="y", alpha=.25); fig.tight_layout()
        fig.savefig(figures/"nominal_accuracy.pdf"); plt.close(fig)
        fig, axes = plt.subplots(1, 3, figsize=(10, 3.6))
        for ax, metric, title, reference in zip(axes,
                ("position_nees_mean", "position_covariance_coverage_95", "hpl_containment"),
                ("Mean position NEES", "95% covariance coverage", "HPL containment"),
                (3.0, .95, 1.0)):
            vals = [modes[k][metric] for k in modes]
            ax.bar(labels, vals, color=[BLUE if k=="0" else ORANGE for k in modes]); ax.axhline(reference, color=BLACK, ls="--")
            ax.set_title(title, fontsize=9); ax.grid(axis="y", alpha=.2)
        fig.tight_layout(); fig.savefig(figures/"consistency.pdf"); plt.close(fig)
    ablation = frames.get("window_ablation")
    if ablation is None:
        placeholder(figures/"window_ablation.pdf", "Window-length ablation", "Not executed")
    else:
        used = ablation[ablation.epoch >= min(200, int(ablation.epoch.max())//5)]
        stats = used.groupby("graph_history").agg(ate=("position_error_m", lambda x: np.sqrt(np.mean(x*x))),
                    latency=("core_ms", lambda x: np.quantile(x,.99)), factors=("active_factors","max")).reset_index()
        fig, ax = plt.subplots(figsize=(7,4)); size=30+130*stats.factors/stats.factors.max()
        ax.scatter(stats.latency, stats.ate, s=size, c=[BLUE if h==0 else ORANGE for h in stats.graph_history])
        for _, row in stats.iterrows(): ax.annotate("full" if row.graph_history==0 else str(int(row.graph_history)), (row.latency,row.ate), xytext=(4,4), textcoords="offset points")
        ax.set(xlabel="P99 core latency [ms]", ylabel="ATE RMSE [m]"); ax.grid(alpha=.25)
        fig.tight_layout(); fig.savefig(figures/"window_ablation.pdf"); plt.close(fig)
    performance = frames.get("performance")
    if performance is None:
        for name, title in (("latency", "Latency comparison"), ("timing_breakdown", "Timing breakdown"), ("resource_growth", "Resource growth")):
            placeholder(figures/f"{name}.pdf", title, "Profiling campaign not executed")
    else:
        fig, axes = plt.subplots(1,2,figsize=(10,3.8))
        for history,color,label in ((0,BLUE,"full"),(200,ORANGE,"lag 200")):
            rows=performance[performance.graph_history==history]
            if rows.empty: continue
            axes[0].plot(rows.epoch, rows.core_ms, color=color, alpha=.7, label=label)
            axes[1].plot(np.sort(rows.core_ms), np.linspace(0,1,len(rows)), color=color,label=label)
        axes[0].set(xlabel="epoch",ylabel="core [ms]"); axes[1].set(xlabel="core [ms]",ylabel="empirical CDF")
        for ax in axes: ax.grid(alpha=.2); ax.legend()
        fig.tight_layout(); fig.savefig(figures/"latency.pdf"); plt.close(fig)
        stages=["estimator_ms","prior_extraction_ms","integrity_ms","remaining_overhead_ms"]
        labels=["Estimator","Prior extraction","Integrity / PL","Remaining"]
        grouped=performance.groupby("graph_history")[stages].mean()
        fig,ax=plt.subplots(figsize=(7,4)); bottom=np.zeros(len(grouped))
        for stage,label,color in zip(stages,labels,[BLUE,ORANGE,GREEN,GRAY]):
            ax.bar(["Full" if h==0 else f"Lag {h}" for h in grouped.index], grouped[stage], bottom=bottom,label=label,color=color); bottom+=grouped[stage].to_numpy()
        ax.set_ylabel("Mean non-overlapping time [ms]"); ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(figures/"timing_breakdown.pdf"); plt.close(fig)
        fig,axes=plt.subplots(1,2,figsize=(9,3.8))
        for history,color,label in ((0,BLUE,"full"),(200,ORANGE,"lag 200")):
            rows=performance[performance.graph_history==history]
            axes[0].plot(rows.epoch,rows.active_values,color=color,label=label)
            axes[1].plot(rows.epoch,rows.active_factors,color=color,label=label)
        axes[0].set(xlabel="epoch",ylabel="active values"); axes[1].set(xlabel="epoch",ylabel="active factors")
        for ax in axes: ax.grid(alpha=.2); ax.legend()
        fig.tight_layout(); fig.savefig(figures/"resource_growth.pdf"); plt.close(fig)
    roc = summaries.get("roc")
    if not roc:
        placeholder(figures/"roc.pdf", "Detector ROC / AUC", "Independent calibration and evaluation not executed")
        placeholder(figures/"fault_sensitivity.pdf", "Fault sensitivity", "Not executed")
    else:
        fig,ax=plt.subplots(figsize=(7,4))
        for history,color in ((0,BLUE),(200,ORANGE)):
            points=[p for p in roc["operating_points"] if p["history"]==history and p["detector"]=="conditional"]
            if points: ax.plot([p["empirical_p_fa"] for p in points],[p["power"] for p in points],marker="o",color=color,label="full" if history==0 else "lag 200")
        ax.set(xscale="log",xlabel="empirical PFA",ylabel="power"); ax.grid(alpha=.2); ax.legend(); fig.tight_layout(); fig.savefig(figures/"roc.pdf"); plt.close(fig)
        fault=frames.get("roc_fault")
        fig,ax=plt.subplots(figsize=(7,4))
        if fault is not None:
            for history,color in ((0,BLUE),(200,ORANGE)):
                rows=fault[(fault.graph_history==history)&(fault.fault_active==1)]
                stats=rows.groupby("fault_m").conditional_passed.apply(lambda x: 1-x.mean())
                ax.plot(stats.index,stats.values,marker="o",color=color,label="full" if history==0 else "lag 200")
        ax.set(xlabel="fault magnitude [m]",ylabel="configured-threshold detection rate"); ax.grid(alpha=.2); ax.legend(); fig.tight_layout(); fig.savefig(figures/"fault_sensitivity.pdf"); plt.close(fig)
    isolated = frames.get("isolated_pl")
    if isolated is None:
        placeholder(figures/"isolated_pl_challenge.pdf", "Single-epoch PL challenge",
                    "Complete 640-seed matrix not executed")
    else:
        active = isolated[isolated.fault_active.astype(bool)].copy()
        active["ratio"] = active.conditional_statistic / active.conditional_threshold
        fig, axes = plt.subplots(1, 3, figsize=(12, 3.9))
        for history, color, marker, label in ((0, BLUE, "o", "full"),
                                               (200, ORANGE, "x", "lag 200")):
            rows = active[active.graph_history == history]
            background = rows[rows.trajectory != "figure_eight"]
            highlighted = rows[rows.trajectory == "figure_eight"]
            axes[0].scatter(background.fault_m, background.ratio, c=color,
                            marker=marker, alpha=.12, s=11)
            axes[0].scatter(highlighted.fault_m, highlighted.ratio, c=color,
                            marker=marker, alpha=.8, s=16, label=f"{label}, figure-eight")
            finite = rows[rows.finite_pl.astype(bool)]
            axes[1].scatter(finite.hpl_m, finite.horizontal_error_m, c=color,
                            marker=marker, alpha=.42, s=13, label=label)
            axes[2].scatter(finite.vpl_m, finite.vertical_error_m, c=color,
                            marker=marker, alpha=.42, s=13, label=label)
        axes[0].axhline(1, color=BLACK, ls="--", lw=1)
        axes[0].set(xlabel="fault magnitude [m]", ylabel=r"$T/\gamma$")
        for ax, xlabel, ylabel in ((axes[1], "HPL [m]", "horizontal error [m]"),
                                   (axes[2], "VPL [m]", "vertical error [m]")):
            limits = [0, max(ax.get_xlim()[1], ax.get_ylim()[1])]
            ax.plot(limits, limits, color=BLACK, ls="--", lw=1)
            ax.set(xlabel=xlabel, ylabel=ylabel, xlim=limits, ylim=limits)
        # Availability fraction by magnitude is overlaid on the vertical panel.
        fractions = active.groupby(["fault_m", "availability"]).size().unstack(fill_value=0)
        fractions = fractions.div(fractions.sum(axis=1), axis=0)
        inset = axes[2].inset_axes([.50, .53, .47, .43])
        bottom = np.zeros(len(fractions))
        for availability_name, color in (("AVAILABLE", GREEN), ("ALERT", RED),
                                          ("UNAVAILABLE", GRAY)):
            values = fractions.get(availability_name, pd.Series(0, index=fractions.index)).to_numpy()
            inset.bar(np.arange(len(fractions)), values, bottom=bottom, color=color,
                      width=.8, label=availability_name)
            bottom += values
        inset.set(xticks=np.arange(len(fractions)),
                  xticklabels=[f"{value:g}" for value in fractions.index],
                  ylim=(0, 1), title="availability by fault [m]")
        inset.tick_params(labelsize=5); inset.title.set_fontsize(6)
        inset.legend(fontsize=4, loc="lower right")
        for ax in axes:
            ax.grid(alpha=.2); ax.legend(fontsize=7)
        fig.tight_layout(); fig.savefig(figures/"isolated_pl_challenge.pdf"); plt.close(fig)

    persistent = frames.get("persistent_fault")
    if persistent is None:
        placeholder(figures/"persistent_fault_timeline.pdf", "Persistent-fault policy",
                    "Complete 8-anchor validation not executed")
    else:
        spec = summaries.get("persistent_fault", {}).get("representative", {})
        seed = spec.get("seed", int(persistent.seed.min()))
        representative = persistent[(persistent.seed == seed) &
                                    (persistent.trajectory == "figure_eight")].copy()
        representative = representative.sort_values(["graph_history", "epoch"])
        fig, axes = plt.subplots(4, 1, figsize=(10, 7), sharex=True)
        onset = int(persistent[persistent.fault_active.astype(bool)].epoch.min())
        for history, color, label in ((0, BLUE, "full"), (200, ORANGE, "lag 200")):
            rows = representative[representative.graph_history == history]
            axes[0].plot(rows.epoch, rows.conditional_statistic, color=color, label=f"{label} T")
            axes[0].plot(rows.epoch, rows.conditional_threshold, color=color, ls="--",
                         alpha=.65, label=f"{label} threshold")
            axes[1].step(rows.epoch, rows.batch_committed.astype(int), where="mid",
                         color=color, label=f"{label} commit")
            availability_colors = rows.availability.map(
                {"AVAILABLE": GREEN, "ALERT": RED, "UNAVAILABLE": GRAY})
            band_y = 1.17 if history == 0 else -.17
            axes[1].scatter(rows.epoch, np.full(len(rows), band_y),
                            c=availability_colors, marker="s", s=5, linewidths=0,
                            label=f"{label} availability band")
            axes[2].plot(rows.epoch, rows.horizontal_error_m, color=color, label=f"{label} error")
            axes[2].plot(rows.epoch, rows.hpl_m, color=color, ls="--", label=f"{label} HPL")
            axes[3].plot(rows.epoch, rows.vertical_error_m, color=color, label=f"{label} error")
            axes[3].plot(rows.epoch, rows.vpl_m, color=color, ls="--", label=f"{label} VPL")
        alert_epochs = representative[representative.availability == "ALERT"].epoch.unique()
        for epoch in alert_epochs:
            for ax in axes:
                ax.axvspan(epoch-.5, epoch+.5, color=RED, alpha=.035, lw=0)
        for ax in axes:
            ax.axvline(onset, color=RED, ls=":", lw=1); ax.grid(alpha=.2)
            ax.legend(fontsize=6, ncol=2, loc="upper left")
        axes[0].set_ylabel("statistic")
        axes[1].set(ylabel="commit", yticks=[0, 1], ylim=(-.3, 1.3))
        axes[1].text(.99, .08, "band: green AVAILABLE, red ALERT, gray UNAVAILABLE",
                     transform=axes[1].transAxes, ha="right", color=RED, fontsize=7)
        axes[2].set_ylabel("horizontal [m]")
        axes[3].set(ylabel="vertical [m]", xlabel="epoch")
        axes[2].text(.99, .08, "blank PL during ALERT is not zero",
                     transform=axes[2].transAxes, ha="right", color=RED, fontsize=7)
        fig.tight_layout(); fig.savefig(figures/"persistent_fault_timeline.pdf"); plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,3.8)); ax.axis("off")
    gates=week4.get("required_gates",{})
    ordered=["noncentral","imu_monte_carlo","method_ab_shadow","ros_topic_tests","performance","snapshot_sweep","roc","history_experiments"]
    for i,name in enumerate(ordered):
        status=gates.get(name,"MISSING"); color=GREEN if status=="PASS" else (RED if status=="FAIL" else GRAY)
        x=.02+(i%4)*.245; y=.62-(i//4)*.42
        ax.add_patch(FancyBboxPatch((x,y),.21,.24,boxstyle="round,pad=.015",facecolor=color,alpha=.9,edgecolor="none"))
        ax.text(x+.105,y+.13,name.replace("_","\n"),ha="center",va="center",color="white",fontsize=9,weight="bold")
        ax.text(x+.105,y+.035,status,ha="center",va="center",color="white",fontsize=8)
    fig.tight_layout(); fig.savefig(figures/"validation_dashboard.pdf"); plt.close(fig)


def latex_escape(value: str) -> str:
    return value.replace("_", "\\_").replace("%", "\\%")


def macros(path: pathlib.Path, summary: dict[str, Any]) -> None:
    nominal = summary.get("nominal", {}).get("modes", {})
    def metric(history: str, key: str, fmt: str = ".3f") -> str:
        value = nominal.get(history, {}).get(key)
        return "N/A" if value is None else format(value, fmt)
    paired = summary.get("nominal", {}).get("paired_fixed_lag_vs_full_history", {})
    roc = summary.get("roc", {}).get("detectors", {})
    isolated = summary.get("isolated_pl", {})
    isolated_overall = isolated.get("overall", {})
    persistent = summary.get("persistent_fault", {})
    inventory = summary.get("inventory", {})
    week4 = summary["week4"]
    content = [
        f"\\newcommand{{\\ExecutionProfile}}{{{latex_escape(summary['execution_profile'])}}}",
        f"\\newcommand{{\\ArtifactGit}}{{{summary['git_sha'][:12]}}}",
        f"\\newcommand{{\\ProtocolHash}}{{{summary['protocol_sha256'][:12]}}}",
        f"\\newcommand{{\\WeekFourStatus}}{{{latex_escape(week4.get('status','MISSING'))}}}",
        f"\\newcommand{{\\FullATE}}{{{metric('0','ate_rmse_m')}}}",
        f"\\newcommand{{\\LagATE}}{{{metric('200','ate_rmse_m')}}}",
        f"\\newcommand{{\\FullPnn}}{{{metric('0','position_error_p99_m')}}}",
        f"\\newcommand{{\\LagPnn}}{{{metric('200','position_error_p99_m')}}}",
        f"\\newcommand{{\\FullLatency}}{{{metric('0','core_mean_ms','.2f')}}}",
        f"\\newcommand{{\\LagLatency}}{{{metric('200','core_mean_ms','.2f')}}}",
        f"\\newcommand{{\\FullNEES}}{{{metric('0','position_nees_mean')}}}",
        f"\\newcommand{{\\LagNEES}}{{{metric('200','position_nees_mean')}}}",
        f"\\newcommand{{\\PairedBlocks}}{{{paired.get('blocks','N/A')}}}",
        f"\\newcommand{{\\PairedCIUpper}}{{{format(100*paired['ci95'][1],'.2f') if 'ci95' in paired else 'N/A'}}}",
        f"\\newcommand{{\\PairedVerdict}}{{{'PASS' if paired.get('criterion_ci_upper_le_5pct') else ('FAIL' if paired.get('status')=='AVAILABLE' else 'N/A')}}}",
        f"\\newcommand{{\\NominalRows}}{{{inventory.get('nominal',{}).get('rows','N/A')}}}",
        f"\\newcommand{{\\RocFaultRows}}{{{inventory.get('roc_fault',{}).get('rows','N/A')}}}",
        f"\\newcommand{{\\FullFactors}}{{{nominal.get('0',{}).get('active_factors_max','N/A')}}}",
        f"\\newcommand{{\\LagFactors}}{{{nominal.get('200',{}).get('active_factors_max','N/A')}}}",
        f"\\newcommand{{\\FullOrientation}}{{{metric('0','orientation_rmse_rad')}}}",
        f"\\newcommand{{\\LagOrientation}}{{{metric('200','orientation_rmse_rad')}}}",
        f"\\newcommand{{\\FullConditionalAUC}}{{{format(roc.get('0',{}).get('conditional',{}).get('auc'),'.3f') if roc.get('0',{}).get('conditional',{}).get('auc') is not None else 'N/A'}}}",
        f"\\newcommand{{\\LagConditionalAUC}}{{{format(roc.get('200',{}).get('conditional',{}).get('auc'),'.3f') if roc.get('200',{}).get('conditional',{}).get('auc') is not None else 'N/A'}}}",
        f"\\newcommand{{\\PFAHolmVerdict}}{{{'PASS' if summary.get('roc',{}).get('pfa_holm_pass') else ('FAIL' if 'roc' in summary else 'N/A')}}}",
        f"\\newcommand{{\\PowerNIVerdict}}{{{'PASS' if summary.get('roc',{}).get('conditional_noninferiority',{}).get('pass') else ('FAIL' if summary.get('roc',{}).get('conditional_noninferiority',{}).get('ci95') else 'N/A')}}}",
        f"\\newcommand{{\\IsolatedSamples}}{{{isolated.get('fault_epoch_samples','N/A')}}}",
        f"\\newcommand{{\\IsolatedFiniteDenominator}}{{{isolated_overall.get('finite_pl_denominator','N/A')}}}",
        f"\\newcommand{{\\IsolatedContainment}}{{{format(100*isolated_overall['containment_rate'],'.2f') if isolated_overall.get('containment_rate') is not None else 'N/A'}}}",
        f"\\newcommand{{\\IsolatedHMI}}{{{isolated_overall.get('hmi_count','N/A')}}}",
        f"\\newcommand{{\\IsolatedHMIUpper}}{{{format(100*isolated_overall['hmi_cp95_upper'],'.3f') if isolated_overall.get('hmi_cp95_upper') is not None else 'N/A'}}}",
        f"\\newcommand{{\\NearBoundarySamples}}{{{isolated.get('near_boundary',{}).get('samples','N/A')}}}",
        f"\\newcommand{{\\PersistentDetection}}{{{format(100*persistent['detection_rate'],'.2f') if persistent.get('detection_rate') is not None else 'N/A'}}}",
        f"\\newcommand{{\\PersistentMaxReject}}{{{persistent.get('maximum_continuous_rejection_epochs','N/A')}}}",
        f"\\newcommand{{\\PersistentAlert}}{{{format(100*persistent['alert_rate'],'.2f') if persistent.get('alert_rate') is not None else 'N/A'}}}",
        f"\\newcommand{{\\PersistentUnavailable}}{{{format(100*persistent['unavailable_rate'],'.2f') if persistent.get('unavailable_rate') is not None else 'N/A'}}}",
        f"\\newcommand{{\\PersistentDrift}}{{{format(persistent['maximum_imu_only_drift_m'],'.3f') if persistent.get('maximum_imu_only_drift_m') is not None else 'N/A'}}}",
        f"\\newcommand{{\\PersistentViolations}}{{{persistent.get('rejected_with_finite_pl_violations','N/A')}}}",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(content)+"\n", encoding="utf-8")


def latest_week4(repository: pathlib.Path) -> dict[str, Any]:
    candidates=[]
    for name in glob.glob(str(repository/"results/week4_*/acceptance_summary.json")):
        path=pathlib.Path(name)
        try:
            value=json.loads(path.read_text()); value["source"]=str(path.relative_to(repository))
            candidates.append((path.stat().st_mtime,value,path.parent))
        except (OSError,json.JSONDecodeError): pass
    if not candidates: return {"status":"MISSING","required_gates":{}}
    _,value,root=max(candidates,key=lambda item:item[0])
    ok,errors=verify_checksums(root)
    value["checksum_reverified"]=ok; value["checksum_errors"]=errors
    return value


def main() -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("root",type=pathlib.Path)
    parser.add_argument("--protocol",type=pathlib.Path,required=True); args=parser.parse_args()
    repository=pathlib.Path(__file__).resolve().parents[1]
    protocol=load_protocol(args.protocol); root=args.root.resolve()
    manifest=json.loads((root/"experiment_manifest.json").read_text())
    raw_names=["nominal","window_ablation","roc_calibration","roc_nominal","roc_fault",
               "representative_fault","isolated_pl","persistent_fault",
               "representative_trajectory"]
    frames={name:read(root/"raw"/f"{name}.csv") for name in raw_names}
    perf_files=sorted((root/"raw").glob("performance_h*_r*.csv"))
    frames["performance"]=pd.concat([pd.read_csv(path).assign(source_run=path.stem) for path in perf_files],ignore_index=True) if perf_files else None
    def expected_range(name: str) -> tuple[int, int] | None:
        suite = manifest.get("suites", {}).get(name, {})
        runs = suite.get("runs", []) if isinstance(suite, dict) else []
        ranges = {tuple(run["output_epochs"]) for run in runs if "output_epochs" in run}
        return next(iter(ranges)) if len(ranges) == 1 else None
    inventory={name:validate(frame,name,expected_range(name))
               for name,frame in frames.items() if frame is not None}
    profile=manifest.get("execution_profile") or "NOT_EXECUTED"
    trials=1000 if profile=="SMOKE" else int(protocol["report"]["bootstrap_trials"])
    summary={"schema_version":"uwb-imu-pl/advisor-report-summary/v1","execution_profile":profile,
             "scientific_claim_scope":"deterministic simulation and component validation only",
             "git_sha":manifest["git_sha"],"protocol_sha256":manifest["protocol_sha256"],
             "inventory":inventory,"week4":latest_week4(repository),"claims":protocol["claims"]}
    if frames["nominal"] is not None:
        summary["nominal"]=nominal_summary(frames["nominal"],protocol["report"]["warmup_epochs"],trials,protocol["report"]["bootstrap_seed"])
    if all(frames[name] is not None for name in ("roc_calibration","roc_nominal","roc_fault")):
        seeds=[set(frames[name].seed.unique()) for name in ("roc_calibration","roc_nominal","roc_fault")]
        if seeds[0]&seeds[1] or seeds[0]&seeds[2] or seeds[1]&seeds[2]: raise ValueError("ROC seed leakage")
        summary["roc"]=roc_summary(frames["roc_calibration"],frames["roc_nominal"],frames["roc_fault"],protocol,profile,trials,protocol["report"]["bootstrap_seed"])
    if frames["isolated_pl"] is not None:
        summary["isolated_pl"], isolated_table = isolated_pl_summary(
            frames["isolated_pl"], protocol)
        isolated_table.to_csv(root/"summary/isolated_pl_cells.csv", index=False)
    if frames["persistent_fault"] is not None:
        summary["persistent_fault"], persistent_table = persistent_fault_summary(
            frames["persistent_fault"])
        summary["persistent_fault"]["representative"] = protocol[
            "persistent_fault"]["representative"]
        persistent_table.to_csv(root/"summary/persistent_fault_sequences.csv", index=False)
    atomic_json(root/"summary/summary.json",summary)
    # Compact machine-readable tables are derived from exactly the same summary.
    rows=[]
    for history,values in summary.get("nominal",{}).get("modes",{}).items(): rows.append({"graph_history":history,**values})
    if rows: pd.DataFrame(rows).to_csv(root/"summary/nominal_modes.csv",index=False)
    figures=repository/"doc/advisor_report/figures"; make_figures(figures,frames,summary,summary["week4"])
    # Copy figure artifacts into the immutable run namespace as well.
    target=root/"figures"; target.mkdir(exist_ok=True)
    for source in figures.glob("*.pdf"): shutil.copy2(source,target/source.name)
    macros(repository/"doc/advisor_report/data/results_macros.tex",summary)
    shutil.copy2(root/"summary/summary.json",repository/"doc/advisor_report/data/summary.json")
    print(json.dumps({"profile":profile,"inventory":inventory,"week4":summary["week4"].get("status")},sort_keys=True))
    return 0


if __name__ == "__main__": raise SystemExit(main())
