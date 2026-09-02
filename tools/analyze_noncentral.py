#!/usr/bin/env python3
"""Week-4 noncentral Monte-Carlo global and multiplicity gates."""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
from typing import Any

import numpy as np

from week4_common import (FAIL, INVALID, PASS, InvalidArtifact, atomic_json,
                          exact_binomial_probability_ordering, holm,
                          load_protocol)

try:
    from scipy.stats import binomtest
except ImportError:  # pragma: no cover - fallback for minimal installations
    binomtest = None


def read_rows(path: pathlib.Path) -> list[dict[str, str]]:
    try:
        with path.open(newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))
    except OSError as error:
        raise InvalidArtifact(f"cannot read {path}: {error}") from error


def exact_p(successes: int, trials: int, probability: float) -> float:
    if binomtest is not None:
        return float(binomtest(successes, trials, probability,
                              alternative="two-sided").pvalue)
    return exact_binomial_probability_ordering(successes, trials, probability)


def analyze(h0_path: pathlib.Path, noncentral_path: pathlib.Path,
            protocol: dict[str, Any], bootstrap_override: int | None = None
            ) -> dict[str, Any]:
    spec = protocol["noncentral"]
    h0 = read_rows(h0_path)
    alternatives = read_rows(noncentral_path)
    expected_h0 = len(spec["h0"]["anchor_counts"]) * len(spec["h0"]["dofs"])
    expected_alt = (len(spec["alternatives"]["dofs"]) *
                    len(spec["alternatives"]["anchor_ids"]) *
                    len(spec["alternatives"]["eta_ratios"]))
    if len(h0) != expected_h0 or len(alternatives) != expected_alt:
        raise InvalidArtifact(
            f"row count mismatch: h0={len(h0)}/{expected_h0}, "
            f"noncentral={len(alternatives)}/{expected_alt}")
    h0_keys = set()
    h0_pass = True
    for row in h0:
        try:
            key = (int(row["anchor_count"]), int(row["dof"]))
            values = [float(row[name]) for name in (
                "p_fa", "threshold", "empirical_p_fa", "ci95_low",
                "ci95_high", "ks", "ks_p")]
            trials = int(row["trials"])
            target_in_ci = row["target_in_ci"].lower() in ("1", "true")
        except (KeyError, ValueError) as error:
            raise InvalidArtifact(f"invalid H0 row: {error}") from error
        if key in h0_keys or not all(math.isfinite(x) for x in values):
            raise InvalidArtifact("duplicate or nonfinite H0 row")
        h0_keys.add(key)
        if trials != spec["h0"]["trials_per_job"]:
            raise InvalidArtifact("H0 trial count does not match protocol")
        h0_pass &= target_in_ci and float(row["ks_p"]) > spec["h0"]["ks_min_p"]

    alt_keys = set()
    observations, probabilities, trials_vector, p_values = [], [], [], []
    jobs = []
    for row in alternatives:
        try:
            key = (int(row["dof"]), int(row["anchor_id"]),
                   float(row["eta_ratio"]))
            trials = int(row["trials"])
            misses = int(row["misses"])
            probability = float(row["theoretical_p_md"])
        except (KeyError, ValueError) as error:
            raise InvalidArtifact(f"invalid noncentral row: {error}") from error
        if key in alt_keys or not 0 <= misses <= trials or not 0 < probability < 1:
            raise InvalidArtifact("duplicate/invalid noncentral row")
        if trials != spec["alternatives"]["trials_per_job"]:
            raise InvalidArtifact("noncentral trial count does not match protocol")
        alt_keys.add(key)
        p_value = exact_p(misses, trials, probability)
        observations.append(misses)
        probabilities.append(probability)
        trials_vector.append(trials)
        p_values.append(p_value)
        jobs.append({"dof": key[0], "anchor_id": key[1], "eta_ratio": key[2],
                     "misses": misses, "trials": trials,
                     "theoretical_p_md": probability, "exact_p": p_value})

    expected_alt_keys = {(int(dof), int(anchor), float(ratio))
                         for dof in spec["alternatives"]["dofs"]
                         for anchor in spec["alternatives"]["anchor_ids"]
                         for ratio in spec["alternatives"]["eta_ratios"]}
    if alt_keys != expected_alt_keys:
        raise InvalidArtifact("noncentral job matrix does not match protocol")
    correction = holm(p_values,
                      float(spec["multiple_testing"]["familywise_alpha"]))
    for job, item in zip(jobs, correction):
        job["holm"] = item
    rejections = sum(item["rejected"] for item in correction)

    observed = np.asarray(observations, dtype=np.float64)
    probability = np.asarray(probabilities, dtype=np.float64)
    trials_array = np.asarray(trials_vector, dtype=np.int64)
    variance = trials_array * probability * (1.0-probability)
    if np.any(~np.isfinite(variance)) or np.any(variance <= 0):
        raise InvalidArtifact("noncentral Pearson expected variance is invalid")
    statistic = float(np.sum((observed-trials_array*probability)**2/variance))
    bootstrap_trials = int(bootstrap_override or
                           spec["global_test"]["parameter_bootstrap_trials"])
    if bootstrap_trials <= 0:
        raise InvalidArtifact("bootstrap trial count must be positive")
    rng = np.random.default_rng(int(spec["root_seed"]))
    exceed = 0
    generated = 0
    chunk_size = min(2000, bootstrap_trials)
    while generated < bootstrap_trials:
        count = min(chunk_size, bootstrap_trials-generated)
        draws = rng.binomial(trials_array, probability, size=(count, len(jobs)))
        boot = np.sum((draws-trials_array*probability)**2/variance, axis=1)
        exceed += int(np.count_nonzero(boot >= statistic))
        generated += count
    global_p = (exceed+1)/(bootstrap_trials+1)
    checks = {
        "h0_exact_ci_and_ks": bool(h0_pass),
        "global_bootstrap_p": global_p >= spec["global_test"]["min_p"],
        "holm_zero_rejections": rejections <=
            spec["multiple_testing"]["maximum_rejections"],
        "finite_complete": True,
    }
    return {
        "status": PASS if all(checks.values()) else FAIL,
        "checks": checks, "h0_jobs": len(h0), "noncentral_jobs": len(jobs),
        "global": {"pearson": statistic, "bootstrap_p": global_p,
                   "bootstrap_trials": bootstrap_trials},
        "holm_rejections": rejections, "jobs": jobs,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("h0_csv", type=pathlib.Path)
    parser.add_argument("noncentral_csv", type=pathlib.Path)
    parser.add_argument("output_json", type=pathlib.Path)
    parser.add_argument("--protocol", type=pathlib.Path,
                        default=pathlib.Path("config/week4_validation.yaml"))
    parser.add_argument("--bootstrap", type=int,
                        help="development fixture override; formal runner rejects it")
    args = parser.parse_args()
    try:
        result = analyze(args.h0_csv, args.noncentral_csv,
                         load_protocol(args.protocol), args.bootstrap)
    except InvalidArtifact as error:
        result = {"status": INVALID, "reason": str(error)}
    atomic_json(args.output_json, result)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == PASS else (1 if result["status"] == FAIL else 2)


if __name__ == "__main__":
    raise SystemExit(main())
