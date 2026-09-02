#!/usr/bin/env python3
"""Seed-level/block-bootstrap summary for snapshot sweep raw epochs."""

import argparse
import csv
import math
import pathlib
import random
from collections import defaultdict


def beta_fraction(a, b, x):
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c = 1.0
    d = 1.0 - qab * x / qap
    d = 1e-300 if abs(d) < 1e-300 else d
    d = 1.0 / d
    h = d
    for m in range(1, 301):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        d = 1e-300 if abs(d) < 1e-300 else d
        c = 1.0 + aa / c
        c = 1e-300 if abs(c) < 1e-300 else c
        d = 1.0 / d
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        d = 1e-300 if abs(d) < 1e-300 else d
        c = 1.0 + aa / c
        c = 1e-300 if abs(c) < 1e-300 else c
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < 3e-14:
            break
    return h


def beta_cdf(x, a, b):
    if x <= 0:
        return 0.0
    if x >= 1:
        return 1.0
    front = math.exp(math.lgamma(a+b) - math.lgamma(a) - math.lgamma(b)
                     + a*math.log(x) + b*math.log1p(-x))
    if x < (a+1)/(a+b+2):
        return front * beta_fraction(a, b, x) / a
    return 1 - front * beta_fraction(b, a, 1-x) / b


def beta_quantile(probability, a, b):
    low, high = 0.0, 1.0
    for _ in range(100):
        middle = (low + high) / 2
        if beta_cdf(middle, a, b) < probability:
            low = middle
        else:
            high = middle
    return (low + high) / 2


def clopper_pearson(successes, trials, alpha=.05, one_sided=False):
    if trials == 0:
        return math.nan, math.nan
    tail = alpha if one_sided else alpha / 2
    low = 0.0 if successes == 0 else beta_quantile(tail, successes, trials-successes+1)
    high = 1.0 if successes == trials else beta_quantile(1-tail, successes+1, trials-successes)
    return low, high


def percentile(values, probability):
    values = sorted(values)
    if not values:
        return math.nan
    position = probability * (len(values)-1)
    left = int(position)
    right = min(left+1, len(values)-1)
    return values[left] + (position-left) * (values[right]-values[left])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input_csv", type=pathlib.Path)
    parser.add_argument("output_csv", type=pathlib.Path)
    parser.add_argument("--bootstrap", type=int, default=2000)
    args = parser.parse_args()
    # Correlated epochs are first collapsed into one record per independent
    # seed/scenario. A seed is the bootstrap block.
    groups = defaultdict(lambda: defaultdict(lambda: {
        "epochs": 0, "covered": 0, "hmi": 0,
        "available": 0, "alert": 0, "unavailable": 0,
        "error_sum": [0.0, 0.0, 0.0],
        "error_square_sum": [0.0, 0.0, 0.0],
        "predicted_variance_sum": [0.0, 0.0, 0.0]}))
    key_fields = ("trajectory", "geometry", "anchor_count", "geometry_scale",
                  "sigma", "fault_anchor", "fault_m")
    with args.input_csv.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            key = tuple(row[field] for field in key_fields)
            seed = row["seed"]
            item = groups[key][seed]
            item["epochs"] += 1
            item["covered"] += row["axis_covered"] in ("1", "true", "True")
            item["hmi"] += row["operational_hmi"] in ("1", "true", "True")
            for axis, suffix in enumerate(("x", "y", "z")):
                error = float(row[f"signed_pe_{suffix}"])
                item["error_sum"][axis] += error
                item["error_square_sum"][axis] += error * error
                item["predicted_variance_sum"][axis] += float(row[f"pred_var_{suffix}"])
            availability = row["availability"].lower()
            if availability == "available": item["available"] += 1
            elif availability == "alert": item["alert"] += 1
            else: item["unavailable"] += 1
    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    random_source = random.Random(20260901)
    with args.output_csv.open("w", newline="", encoding="utf-8") as stream:
        fieldnames = list(key_fields) + [
            "independent_seeds", "epochs", "coverage", "coverage_ci95_low",
            "coverage_ci95_high", "seed_bootstrap_ci95_low",
            "seed_bootstrap_ci95_high", "hmi_seed_trials", "hmi_seed_events",
            "hmi_one_sided_95_upper", "available_ratio", "alert_ratio",
            "unavailable_ratio"]
        fieldnames += [f"bias_over_sigma_{axis}" for axis in "xyz"]
        fieldnames += [f"empirical_predicted_variance_ratio_{axis}" for axis in "xyz"]
        fieldnames += ["small_noise_covariance_pass"]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for key in sorted(groups):
            seeds = list(groups[key].values())
            epochs = sum(item["epochs"] for item in seeds)
            covered = sum(item["covered"] for item in seeds)
            seed_covered = sum(item["covered"] == item["epochs"] for item in seeds)
            coverage_low, coverage_high = clopper_pearson(seed_covered, len(seeds))
            seed_coverage = [item["covered"] / item["epochs"] for item in seeds]
            bootstrap = []
            for _ in range(args.bootstrap):
                sample = [random_source.choice(seed_coverage) for _ in seed_coverage]
                bootstrap.append(sum(sample) / len(sample))
            hmi_events = sum(item["hmi"] > 0 for item in seeds)
            _, hmi_upper = clopper_pearson(hmi_events, len(seeds), one_sided=True)
            available = sum(item["available"] for item in seeds)
            alert = sum(item["alert"] for item in seeds)
            unavailable = sum(item["unavailable"] for item in seeds)
            row = dict(zip(key_fields, key))
            row.update({
                "independent_seeds": len(seeds), "epochs": epochs,
                "coverage": covered / epochs,
                "coverage_ci95_low": coverage_low,
                "coverage_ci95_high": coverage_high,
                "seed_bootstrap_ci95_low": percentile(bootstrap, .025),
                "seed_bootstrap_ci95_high": percentile(bootstrap, .975),
                "hmi_seed_trials": len(seeds), "hmi_seed_events": hmi_events,
                "hmi_one_sided_95_upper": hmi_upper,
                "available_ratio": available / epochs,
                "alert_ratio": alert / epochs,
                "unavailable_ratio": unavailable / epochs,
            })
            covariance_pass = True
            for axis, suffix in enumerate("xyz"):
                error_sum = sum(item["error_sum"][axis] for item in seeds)
                square_sum = sum(item["error_square_sum"][axis] for item in seeds)
                predicted = sum(item["predicted_variance_sum"][axis]
                                for item in seeds) / epochs
                mean = error_sum / epochs
                empirical = max(0.0, square_sum / epochs - mean * mean)
                bias_over_sigma = abs(mean) / math.sqrt(predicted) if predicted > 0 else math.inf
                ratio = empirical / predicted if predicted > 0 else math.inf
                row[f"bias_over_sigma_{suffix}"] = bias_over_sigma
                row[f"empirical_predicted_variance_ratio_{suffix}"] = ratio
                covariance_pass &= bias_over_sigma < .1 and .8 <= ratio <= 1.2
            # The field is meaningful as a calibration gate only for nominal
            # Gaussian rows; faulted/stress rows are retained but marked false.
            row["small_noise_covariance_pass"] = (
                covariance_pass and key[key_fields.index("fault_anchor")] == "-1"
                and key[key_fields.index("geometry")] == "regular")
            writer.writerow(row)


if __name__ == "__main__":
    main()
