#!/usr/bin/env python3
"""Preregistered detector ROC/history analysis with sequence-block bootstrap."""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import pathlib
from collections import defaultdict
from typing import Any, Iterable

import numpy as np
from scipy.stats import chi2

from week4_common import (FAIL, INVALID, PASS, InvalidArtifact, atomic_json,
                          holm, load_protocol, percentile, rolling_sum)

DETECTORS = {
    "global": "global_statistic",
    "postfit": "postfit_statistic",
    "conditional": "conditional_statistic",
}


def read_sequences(path: pathlib.Path, expected_seeds: set[int], epochs: int,
                   fault: bool) -> dict[tuple[int, str, int], list[dict[str, str]]]:
    sequences: dict[tuple[int, str, int], list[dict[str, str]]] = defaultdict(list)
    required = {"seed", "trajectory", "graph_history", "epoch", *DETECTORS.values()}
    if fault:
        required |= {"fault_active", "fault_m", "anchor_id"}
    paths = sorted(path.glob("*.csv")) if path.is_dir() else [path]
    if not paths:
        raise InvalidArtifact(f"{path}: no CSV shards")
    for shard in paths:
        try:
            stream = shard.open(newline="", encoding="utf-8")
        except OSError as error:
            raise InvalidArtifact(f"cannot open {shard}: {error}") from error
        with stream:
            reader = csv.DictReader(stream)
            missing = required - set(reader.fieldnames or [])
            if missing:
                raise InvalidArtifact(f"{shard}: missing fields {sorted(missing)}")
            for line, row in enumerate(reader, 2):
                try:
                    seed = int(row["seed"])
                    history = int(row["graph_history"])
                    epoch = int(row["epoch"])
                    values = [float(row[field]) for field in DETECTORS.values()]
                except ValueError as error:
                    raise InvalidArtifact(f"{shard}:{line}: {error}") from error
                if seed not in expected_seeds or epoch < 0 or not all(
                        math.isfinite(value) for value in values):
                    raise InvalidArtifact(f"{shard}:{line}: unregistered/nonfinite row")
                sequences[(seed, row["trajectory"], history)].append(row)
    if not sequences:
        raise InvalidArtifact(f"{path}: no sequences")
    for block, rows in sequences.items():
        if len(rows) != epochs:
            raise InvalidArtifact(f"{path}: sequence {block} has {len(rows)}/{epochs} rows")
        actual = [int(row["epoch"]) for row in rows]
        if actual != list(range(epochs)):
            raise InvalidArtifact(f"{path}: duplicate, missing, or unordered epochs in {block}")
    return sequences


def seed_set(spec: dict[str, Any], name: str) -> set[int]:
    definition = spec[name]
    return set(range(int(definition["start"]),
                     int(definition["start"])+int(definition["count"])))


def empirical_threshold(values: Iterable[float], p_fa: float) -> float:
    ordered = sorted(value for value in values if math.isfinite(value))
    if not ordered:
        raise InvalidArtifact("empty calibration sample")
    return ordered[min(len(ordered)-1, math.ceil((1-p_fa)*len(ordered))-1)]


def block_bootstrap_pfa(blocks: list[list[bool]], target: float, trials: int,
                        rng: np.random.Generator) -> tuple[float, float, float, float]:
    if not blocks or any(not block for block in blocks):
        raise InvalidArtifact("empty evaluation bootstrap block")
    rates = np.asarray([sum(block)/len(block) for block in blocks], dtype=float)
    weights = np.asarray([len(block) for block in blocks], dtype=float)
    observed = float(sum(map(sum, blocks))/sum(map(len, blocks)))
    draws = np.empty(trials)
    for index in range(trials):
        selected = rng.integers(0, len(blocks), len(blocks))
        draws[index] = float(np.sum(rates[selected]*weights[selected]) /
                             np.sum(weights[selected]))
    # Recenter the empirical block distribution at the preregistered maximum.
    # A detector more conservative than the target is not a PFA failure.
    null_draws = draws - observed + target
    p_value = 1.0 if observed <= target else (
        np.count_nonzero(null_draws-target >= observed-target)+1)/(trials+1)
    return observed, float(np.quantile(draws, .025)), \
        float(np.quantile(draws, .975)), float(p_value)


def auc(nominal: list[float], faulted: list[float]) -> float:
    if not nominal or not faulted:
        return math.nan
    # Pairwise definition with tie half-credit, evaluated by sorted search.
    ordered = np.sort(np.asarray(nominal))
    wins = 0.0
    for value in faulted:
        left = int(np.searchsorted(ordered, value, side="left"))
        right = int(np.searchsorted(ordered, value, side="right"))
        wins += left + .5*(right-left)
    return wins/(len(nominal)*len(faulted))


def write_complete_roc(stream, history: int, window: int, detector: str,
                       nominal: list[float], faulted: list[float]) -> int:
    tagged = [(value, 0) for value in nominal] + [(value, 1) for value in faulted]
    tagged.sort(reverse=True)
    false_positive = true_positive = points = 0
    index = 0
    stream.write(f"{history},{window},{detector},inf,0,0\n")
    while index < len(tagged):
        threshold = tagged[index][0]
        while index < len(tagged) and tagged[index][0] == threshold:
            if tagged[index][1]: true_positive += 1
            else: false_positive += 1
            index += 1
        stream.write(f"{history},{window},{detector},{threshold:.17g},"
                     f"{false_positive/len(nominal):.17g},"
                     f"{true_positive/len(faulted):.17g}\n")
        points += 1
    return points + 1


def analyze(calibration_path: pathlib.Path, nominal_path: pathlib.Path,
            fault_path: pathlib.Path, protocol: dict[str, Any],
            bootstrap_override: int | None = None,
            roc_curve_path: pathlib.Path | None = None) -> dict[str, Any]:
    spec = protocol["roc"]
    calibration_seeds = seed_set(spec, "calibration_seeds")
    nominal_seeds = seed_set(spec, "nominal_evaluation_seeds")
    fault_seeds = seed_set(spec, "fault_evaluation_seeds")
    if calibration_seeds & nominal_seeds or calibration_seeds & fault_seeds or \
            nominal_seeds & fault_seeds:
        raise InvalidArtifact("protocol calibration/evaluation seed leakage")
    epochs = int(spec["epochs_per_sequence"])
    calibration = read_sequences(calibration_path, calibration_seeds, epochs, False)
    nominal = read_sequences(nominal_path, nominal_seeds, epochs, False)
    fault = read_sequences(fault_path, fault_seeds, epochs, True)
    histories = {int(value) for value in spec["graph_histories"]}
    trajectories = set(spec["trajectories"])
    for name, sequences, seeds in (("calibration", calibration, calibration_seeds),
                                   ("nominal", nominal, nominal_seeds),
                                   ("fault", fault, fault_seeds)):
        actual = set(sequences)
        expected = {(seed, trajectory, history) for seed in seeds
                    for trajectory in trajectories for history in histories}
        if actual != expected:
            missing = len(expected-actual)
            extra = len(actual-expected)
            raise InvalidArtifact(f"{name} sequence matrix mismatch missing={missing} extra={extra}")

    trials = int(bootstrap_override or spec["bootstrap"]["trials"])
    rng = np.random.default_rng(int(spec["bootstrap"]["seed"]))
    operating = []
    p_values = []
    power_blocks: dict[tuple[int, str, float], dict[tuple[int, str], float]] = {}
    roc_summaries = []
    curve_stream = None
    if roc_curve_path:
        roc_curve_path.parent.mkdir(parents=True, exist_ok=True)
        curve_stream = gzip.open(roc_curve_path, "wt", encoding="utf-8", newline="")
        curve_stream.write("graph_history,rolling_window,detector,threshold,false_positive_rate,true_positive_rate\n")
    for history in sorted(histories):
        windows = {1}
        if history == int(spec["temporal_ablation"]["graph_history"]):
            windows.update(map(int, spec["temporal_ablation"]["rolling_windows"]))
        for detector, field in DETECTORS.items():
            for window in sorted(windows):
                cal_values = []
                nominal_blocks = []
                fault_values = []
                fault_by_block: dict[tuple[int, str], list[tuple[float, float, int]]] = {}
                for block, rows in calibration.items():
                    if block[2] != history: continue
                    cal_values.extend(value for value in rolling_sum(
                        [float(row[field]) for row in rows], window)
                                      if math.isfinite(value))
                for block, rows in nominal.items():
                    if block[2] != history: continue
                    nominal_blocks.append([value for value in rolling_sum(
                        [float(row[field]) for row in rows], window)
                                           if math.isfinite(value)])
                for block, rows in fault.items():
                    if block[2] != history: continue
                    rolled = rolling_sum([float(row[field]) for row in rows], window)
                    values = []
                    for row, value in zip(rows, rolled):
                        if math.isfinite(value) and row["fault_active"].lower() in ("1", "true"):
                            item = (value, float(row["fault_m"]), int(row["epoch"]))
                            values.append(item)
                            fault_values.append(value)
                    fault_by_block[(block[0], block[1])] = values
                flat_nominal = [value for block in nominal_blocks for value in block]
                curve_points = write_complete_roc(
                    curve_stream, history, window, detector, flat_nominal,
                    fault_values) if curve_stream else 0
                roc_summaries.append({
                    "graph_history": history, "rolling_window": window,
                    "detector": detector, "nominal_count": len(flat_nominal),
                    "fault_count": len(fault_values),
                    "auc": auc(flat_nominal, fault_values),
                    "complete_curve_points": curve_points,
                })
                for target in map(float, spec["operating_p_fa"]):
                    threshold = empirical_threshold(cal_values, target)
                    alarm_blocks = [[value > threshold for value in block]
                                    for block in nominal_blocks]
                    empirical, low, high, p_value = block_bootstrap_pfa(
                        alarm_blocks, target, trials, rng)
                    magnitude_threshold = float(protocol["history_experiments"]
                                                ["gates"]["minimum_fault_magnitude_m"])
                    selected_power = {}
                    detections = total = 0
                    ttd = []
                    onset = int(spec["fault"]["onset_epoch"])
                    for block, values in fault_by_block.items():
                        eligible = [(value, epoch) for value, magnitude, epoch in values
                                    if magnitude >= magnitude_threshold]
                        if not eligible: continue
                        flags = [value > threshold for value, _ in eligible]
                        selected_power[block] = sum(flags)/len(flags)
                        detections += sum(flags)
                        total += len(flags)
                        detected_epochs = [epoch for (value, epoch) in eligible
                                           if value > threshold]
                        if detected_epochs:
                            ttd.append(min(detected_epochs)-onset)
                    if window == int(protocol["history_experiments"]
                                     ["primary_rolling_window"]):
                        power_blocks[(history, detector, target)] = selected_power
                    record = {
                        "graph_history": history, "rolling_window": window,
                        "detector": detector, "target_p_fa": target,
                        "threshold": threshold, "calibration_count": len(cal_values),
                        "evaluation_count": sum(map(len, nominal_blocks)),
                        "empirical_p_fa": empirical, "bootstrap_ci95": [low, high],
                        "bootstrap_p": p_value, "fault_count": total,
                        "power_ge_0_5m": detections/total if total else math.nan,
                        "time_to_detect_median_epochs": percentile(ttd, .5),
                    }
                    operating.append(record)
                    p_values.append(p_value)

    if curve_stream:
        curve_stream.close()
    correction = holm(p_values, float(spec["gates"]["familywise_alpha"]))
    for record, corrected in zip(operating, correction):
        record["holm"] = corrected
    pfa_pass = not any(item["rejected"] for item in correction)
    longest = max(map(int, spec["temporal_ablation"]["rolling_windows"]))
    count_pass = all(record["evaluation_count"] >=
                     spec["gates"]["longest_window_min_valid_epochs_per_item"]
                     for record in operating if record["rolling_window"] == longest)
    theoretical_records = []
    theoretical_p_values = []
    theoretical_target = float(spec["conditional_theoretical_p_fa"])
    theoretical_threshold = float(chi2.ppf(1-theoretical_target, 8))
    for history in sorted(histories):
        blocks = []
        for block, rows in nominal.items():
            if block[2] == history:
                blocks.append([float(row[DETECTORS["conditional"]]) >
                               theoretical_threshold for row in rows])
        empirical, low, high, p_value = block_bootstrap_pfa(
            blocks, theoretical_target, trials, rng)
        theoretical_records.append({"graph_history": history,
            "target_p_fa": theoretical_target, "threshold": theoretical_threshold,
            "evaluation_count": sum(map(len, blocks)), "empirical_p_fa": empirical,
            "bootstrap_ci95": [low, high], "bootstrap_p": p_value})
        theoretical_p_values.append(p_value)
    theoretical_holm = holm(theoretical_p_values,
                            float(spec["gates"]["familywise_alpha"]))
    for item, corrected in zip(theoretical_records, theoretical_holm):
        item["holm"] = corrected
    theoretical_pass = not any(item["rejected"] for item in theoretical_holm)

    history_spec = protocol["history_experiments"]["gates"]
    reference = int(history_spec["reference_graph_history"])
    target = float(history_spec["operating_p_fa"])
    noninferiority = []
    for history in sorted(histories):
        current = power_blocks.get((history, history_spec["detector"], target), {})
        reference_blocks = power_blocks.get((reference, history_spec["detector"], target), {})
        common = sorted(set(current) & set(reference_blocks))
        differences = np.asarray([current[key]-reference_blocks[key] for key in common])
        if not len(differences):
            raise InvalidArtifact("no paired fault blocks for history noninferiority")
        boot = np.empty(trials)
        for index in range(trials):
            boot[index] = float(np.mean(rng.choice(differences, len(differences))))
        lower = float(np.quantile(boot, .025))
        noninferiority.append({"graph_history": history,
                               "mean_power_difference": float(np.mean(differences)),
                               "ci95_lower": lower,
                               "pass": lower > float(history_spec["noninferiority_margin"])})
    history_pass = all(item["pass"] for item in noninferiority)
    checks = {"pfa_holm": pfa_pass, "minimum_evaluation_epochs": count_pass,
              "history_noninferiority": history_pass,
              "conditional_theoretical_pfa": theoretical_pass,
              "calibration_evaluation_disjoint": True,
              "outputs_complete": all(math.isfinite(item["auc"])
                                      for item in roc_summaries)}
    return {"status": PASS if all(checks.values()) else FAIL,
            "checks": checks, "operating_points": operating,
            "roc": roc_summaries, "history_noninferiority": noninferiority,
            "conditional_theoretical": theoretical_records,
            "complete_roc_artifact": str(roc_curve_path) if roc_curve_path else None,
            "bootstrap_trials": trials}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("calibration_csv", type=pathlib.Path)
    parser.add_argument("nominal_evaluation_csv", type=pathlib.Path)
    parser.add_argument("fault_evaluation_csv", type=pathlib.Path)
    parser.add_argument("output_json", type=pathlib.Path)
    parser.add_argument("--protocol", type=pathlib.Path,
                        default=pathlib.Path("config/week4_validation.yaml"))
    parser.add_argument("--bootstrap", type=int)
    args = parser.parse_args()
    try:
        result = analyze(args.calibration_csv, args.nominal_evaluation_csv,
                         args.fault_evaluation_csv, load_protocol(args.protocol),
                         args.bootstrap,
                         args.output_json.with_name("roc_curves.csv.gz"))
    except InvalidArtifact as error:
        result = {"status": INVALID, "reason": str(error)}
    atomic_json(args.output_json, result)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == PASS else (1 if result["status"] == FAIL else 2)


if __name__ == "__main__":
    raise SystemExit(main())
