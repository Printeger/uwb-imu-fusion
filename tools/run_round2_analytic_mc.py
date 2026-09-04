#!/usr/bin/env python3
"""Chunked analytic H0/noncentral Monte Carlo with global bootstrap and Holm."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys

import numpy as np
from scipy import stats

from round2_common import FAIL, PASS, CounterRng, atomic_json, load_protocol


def holm(pvalues: list[float], alpha: float) -> list[bool]:
    result = [False] * len(pvalues)
    for order, index in enumerate(sorted(range(len(pvalues)), key=pvalues.__getitem__)):
        if pvalues[index] <= alpha / (len(pvalues) - order):
            result[index] = True
        else:
            break
    return result


def chunked_exceedances(rng: np.random.Generator, trials: int, dof: int,
                        threshold: float, noncentrality: float = 0.0) -> int:
    exceedances = 0
    remaining = trials
    while remaining:
        count = min(remaining, 250_000)
        values = rng.noncentral_chisquare(dof, noncentrality, count)
        exceedances += int(np.count_nonzero(values > threshold))
        remaining -= count
    return exceedances


def global_bootstrap_p(observed: list[int], trials: list[int],
                       expected: list[float], seed: int,
                       replicates: int) -> float:
    statistic = max(abs(observed[i] / trials[i] - expected[i]) /
                    math.sqrt(max(expected[i] * (1 - expected[i]) / trials[i], 1e-30))
                    for i in range(len(observed)))
    rng = np.random.Generator(np.random.Philox(seed))
    simulated = np.zeros(replicates)
    for i, probability in enumerate(expected):
        draws = rng.binomial(trials[i], probability, replicates)
        standardized = (np.abs(draws / trials[i] - probability) /
                        math.sqrt(max(probability * (1 - probability) /
                                      trials[i], 1e-30)))
        simulated = np.maximum(simulated, standardized)
    return float((1 + np.count_nonzero(simulated >= statistic)) / (replicates + 1))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("protocol", type=pathlib.Path)
    parser.add_argument("output", type=pathlib.Path)
    parser.add_argument("--profile", choices=("full", "smoke"), default="full")
    args = parser.parse_args()
    protocol, protocol_sha = load_protocol(args.protocol)
    count_h0 = protocol["sample_counts"]["analytic_h0_per_dof"]
    count_nc = protocol["sample_counts"]["analytic_noncentral_per_cell"]
    if args.profile == "smoke":
        count_h0, count_nc = 30_000, 10_000
    dofs = [1, 2, 3, 4, 5, 6, 8, 10]
    ratios = protocol["boundary_ratios"]
    p_fa = 1e-3
    p_md = protocol["statistical_gates"]["p_md_boundary"]
    observations, trial_counts, expectations, cells = [], [], [], []
    seed = CounterRng(20260905, "round2", protocol_sha).uint64(0)
    for dof in dofs:
        threshold = float(stats.chi2.ppf(1 - p_fa, dof))
        rng = np.random.Generator(np.random.Philox(seed ^ dof))
        exceed = chunked_exceedances(rng, count_h0, dof, threshold)
        observations.append(exceed); trial_counts.append(count_h0); expectations.append(p_fa)
        cells.append({"family": "H0", "dof": dof, "ratio": 0.0,
                      "trials": count_h0, "exceedances": exceed,
                      "expected": p_fa})
        boundary = 1.0
        while stats.ncx2.cdf(threshold, dof, boundary) > p_md:
            boundary *= 2
        lower = boundary / 2
        for _ in range(100):
            middle = (lower + boundary) / 2
            if stats.ncx2.cdf(threshold, dof, middle) > p_md:
                lower = middle
            else:
                boundary = middle
        lam = (lower + boundary) / 2
        for ratio in ratios:
            noncentrality = ratio * ratio * lam
            expected_miss = float(stats.ncx2.cdf(threshold, dof, noncentrality))
            cell_seed = seed ^ dof ^ int(ratio * 1000)
            rng = np.random.Generator(np.random.Philox(cell_seed))
            detected = chunked_exceedances(rng, count_nc, dof, threshold,
                                           noncentrality)
            misses = count_nc - detected
            observations.append(misses); trial_counts.append(count_nc)
            expectations.append(expected_miss)
            cells.append({"family": "NONCENTRAL", "dof": dof, "ratio": ratio,
                          "trials": count_nc, "misses": misses,
                          "expected": expected_miss, "lambda_boundary": lam})
    individual = [stats.binomtest(observations[i], trial_counts[i],
                                  expectations[i]).pvalue
                  for i in range(len(observations))]
    rejected = holm([float(value) for value in individual],
                    protocol["statistical_gates"]["familywise_alpha"])
    global_p = global_bootstrap_p(observations, trial_counts, expectations,
                                  seed, 10_000 if args.profile == "full" else 1_000)
    status = PASS if global_p >= 0.01 and not any(rejected) else FAIL
    atomic_json(args.output, {
        "schema_version": "uwb-imu-pl/round2-analytic-mc/v1",
        "status": status, "profile": args.profile,
        "protocol_sha256": protocol_sha, "counter_rng": "numpy.Philox",
        "global_parametric_bootstrap_p": global_p,
        "holm_familywise_alpha": 0.01, "holm_rejections": sum(rejected),
        "cells": [{**cell, "exact_binomial_p": float(individual[index]),
                   "holm_rejected": rejected[index]}
                  for index, cell in enumerate(cells)]})
    return 0 if status == PASS else 1


if __name__ == "__main__":
    sys.exit(main())
