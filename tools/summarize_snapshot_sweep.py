#!/usr/bin/env python3
"""Streaming Week-4 snapshot sweep inventory, completeness, and summary."""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import random
from collections import defaultdict
from typing import Any, Iterable

from week4_common import (FAIL, INVALID, PASS, InvalidArtifact, atomic_json,
                          clopper_pearson, load_protocol, percentile,
                          sha256_file)

KEY_FIELDS = ("trajectory", "geometry", "anchor_count", "geometry_scale",
              "sigma", "fault_anchor", "fault_m")
REQUIRED_FIELDS = set(KEY_FIELDS) | {
    "scenario_id", "seed", "epoch", "axis_covered", "operational_hmi",
    "availability", "detector_passed", "model_valid",
    "signed_pe_x", "signed_pe_y", "signed_pe_z",
    "pred_var_x", "pred_var_y", "pred_var_z",
}


def discover(path: pathlib.Path, extras: Iterable[pathlib.Path]) -> list[pathlib.Path]:
    paths = sorted(path.glob("*.csv")) if path.is_dir() else [path]
    paths.extend(extras)
    unique = []
    seen = set()
    for item in paths:
        resolved = item.resolve()
        if resolved not in seen:
            seen.add(resolved)
            unique.append(item)
    if not unique:
        raise InvalidArtifact("no snapshot CSV shards found")
    return unique


def expected_scenarios(spec: dict[str, Any]) -> set[tuple[str, ...]]:
    expected: set[tuple[str, ...]] = set()
    magnitudes = spec["fault"]["magnitudes_m"]
    for trajectory in spec["trajectories"]:
        for anchors in spec["anchor_counts"]:
            faults = [(-1, 0.0)] + [
                (anchor, magnitude) for anchor in range(anchors)
                for magnitude in magnitudes]
            for geometry, geometry_spec in spec["geometries"].items():
                for scale in geometry_spec["geometry_scales"]:
                    for sigma in spec["noise_sigmas_m"]:
                        for anchor, magnitude in faults:
                            expected.add(tuple(map(str, (
                                trajectory, geometry, anchors, scale, sigma,
                                anchor, magnitude))))
    return expected


def normalized_key(row: dict[str, str]) -> tuple[str, ...]:
    return (row["trajectory"], row["geometry"], str(int(row["anchor_count"])),
            str(float(row["geometry_scale"])), str(float(row["sigma"])),
            str(int(row["fault_anchor"])), str(float(row["fault_m"])))


def expected_normalized(spec: dict[str, Any]) -> set[tuple[str, ...]]:
    return {(key[0], key[1], str(int(key[2])), str(float(key[3])),
             str(float(key[4])), str(int(key[5])), str(float(key[6])))
            for key in expected_scenarios(spec)}


def read_and_validate(paths: list[pathlib.Path], spec: dict[str, Any]):
    seed_start = int(spec["seeds"]["start"])
    seed_stop = int(spec["seeds"]["stop"])
    epochs = int(spec["epochs_per_job"])
    expected = expected_normalized(spec)
    seen: dict[tuple[int, tuple[str, ...]], int] = {}
    inventory = []
    total_rows = 0
    duplicate_rows = 0
    bad_rows: list[str] = []
    for path in paths:
        shard_rows = 0
        shard_seeds: set[int] = set()
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            missing = REQUIRED_FIELDS - set(reader.fieldnames or [])
            if missing:
                raise InvalidArtifact(f"{path}: missing columns {sorted(missing)}")
            for line, row in enumerate(reader, 2):
                shard_rows += 1
                total_rows += 1
                try:
                    seed = int(row["seed"])
                    epoch = int(row["epoch"])
                    scenario = normalized_key(row)
                    numeric = [float(row[field]) for field in (
                        "signed_pe_x", "signed_pe_y", "signed_pe_z",
                        "pred_var_x", "pred_var_y", "pred_var_z")]
                except (KeyError, TypeError, ValueError) as error:
                    bad_rows.append(f"{path}:{line}: {error}")
                    continue
                if not seed_start <= seed <= seed_stop or scenario not in expected:
                    bad_rows.append(f"{path}:{line}: unregistered seed/scenario")
                    continue
                if not 0 <= epoch < epochs:
                    bad_rows.append(f"{path}:{line}: invalid epoch")
                    continue
                shard_seeds.add(seed)
                key = (seed, scenario)
                mask = seen.get(key, 0)
                bit = 1 << epoch
                if mask & bit:
                    duplicate_rows += 1
                seen[key] = mask | bit
                yield row
        inventory.append({
            "path": str(path), "bytes": path.stat().st_size,
            "sha256": sha256_file(path), "rows": shard_rows,
            "seed_min": min(shard_seeds) if shard_seeds else None,
            "seed_max": max(shard_seeds) if shard_seeds else None,
            "seed_count": len(shard_seeds),
        })
    expected_mask = (1 << epochs) - 1
    expected_jobs = len(expected) * (seed_stop - seed_start + 1)
    missing_jobs = expected_jobs - len(seen)
    incomplete_jobs = sum(mask != expected_mask for mask in seen.values())
    validation = {
        "status": PASS if not bad_rows and duplicate_rows == 0 and
            missing_jobs == 0 and incomplete_jobs == 0 and
            total_rows == int(spec["expected_rows"]) else INVALID,
        "rows": total_rows, "expected_rows": int(spec["expected_rows"]),
        "jobs": len(seen), "expected_jobs": expected_jobs,
        "duplicate_rows": duplicate_rows, "missing_jobs": missing_jobs,
        "incomplete_jobs": incomplete_jobs, "bad_rows": bad_rows[:100],
        "shards": inventory,
    }
    return validation


def summarize(paths: list[pathlib.Path], protocol: dict[str, Any],
              output_csv: pathlib.Path, inventory_path: pathlib.Path,
              bootstrap_trials: int) -> dict[str, Any]:
    spec = protocol["snapshot_sweep"]
    groups = defaultdict(lambda: defaultdict(lambda: {
        "epochs": 0, "covered": 0, "hmi": 0, "available": 0,
        "alert": 0, "unavailable": 0, "model_valid": 0, "finite": 0,
        "error_sum": [0.0]*3, "error_square_sum": [0.0]*3,
        "predicted_variance_sum": [0.0]*3}))
    generator = read_and_validate(paths, spec)
    try:
        while True:
            row = next(generator)
            key = normalized_key(row)
            item = groups[key][int(row["seed"])]
            item["epochs"] += 1
            item["covered"] += row["axis_covered"].lower() in ("1", "true")
            item["hmi"] += row["operational_hmi"].lower() in ("1", "true")
            item["model_valid"] += row["model_valid"].lower() in ("1", "true")
            numeric = [float(row[f"signed_pe_{suffix}"]) for suffix in "xyz"] + \
                [float(row[f"pred_var_{suffix}"]) for suffix in "xyz"]
            if all(math.isfinite(value) for value in numeric):
                item["finite"] += 1
                for axis, suffix in enumerate("xyz"):
                    error = float(row[f"signed_pe_{suffix}"])
                    item["error_sum"][axis] += error
                    item["error_square_sum"][axis] += error * error
                    item["predicted_variance_sum"][axis] += float(row[f"pred_var_{suffix}"])
            availability = row["availability"].lower()
            if availability == "available": item["available"] += 1
            elif availability == "alert": item["alert"] += 1
            else: item["unavailable"] += 1
    except StopIteration as stop:
        validation = stop.value
    atomic_json(inventory_path, validation)
    if validation["status"] == INVALID:
        raise InvalidArtifact("snapshot sweep completeness check failed")

    output_csv.parent.mkdir(parents=True, exist_ok=True)
    rng = random.Random(int(spec["bootstrap_seed"]))
    records = []
    nominal_regular = []
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        fields = list(KEY_FIELDS) + [
            "independent_seeds", "epochs", "coverage", "coverage_ci95_low",
            "coverage_ci95_high", "seed_bootstrap_ci95_low",
            "seed_bootstrap_ci95_high", "hmi_seed_trials", "hmi_seed_events",
            "hmi_one_sided_95_upper", "available_ratio", "alert_ratio",
            "unavailable_ratio"]
        fields += [f"bias_over_sigma_{axis}" for axis in "xyz"]
        fields += [f"empirical_predicted_variance_ratio_{axis}" for axis in "xyz"]
        fields += ["small_noise_covariance_pass"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for key in sorted(groups):
            seeds = list(groups[key].values())
            epochs = sum(item["epochs"] for item in seeds)
            covered = sum(item["covered"] for item in seeds)
            seed_covered = sum(item["covered"] == item["epochs"] for item in seeds)
            low, high = clopper_pearson(seed_covered, len(seeds))
            seed_rates = [item["covered"] / item["epochs"] for item in seeds]
            bootstrap = [sum(rng.choice(seed_rates) for _ in seed_rates) /
                         len(seed_rates) for _ in range(bootstrap_trials)]
            hmi_events = sum(item["hmi"] > 0 for item in seeds)
            _, hmi_upper = clopper_pearson(hmi_events, len(seeds), one_sided=True)
            row = dict(zip(KEY_FIELDS, key))
            row.update({
                "independent_seeds": len(seeds), "epochs": epochs,
                "coverage": covered/epochs, "coverage_ci95_low": low,
                "coverage_ci95_high": high,
                "seed_bootstrap_ci95_low": percentile(bootstrap, .025),
                "seed_bootstrap_ci95_high": percentile(bootstrap, .975),
                "hmi_seed_trials": len(seeds), "hmi_seed_events": hmi_events,
                "hmi_one_sided_95_upper": hmi_upper,
                "available_ratio": sum(x["available"] for x in seeds)/epochs,
                "alert_ratio": sum(x["alert"] for x in seeds)/epochs,
                "unavailable_ratio": sum(x["unavailable"] for x in seeds)/epochs,
            })
            covariance_pass = True
            finite_epochs = sum(x["finite"] for x in seeds)
            covariance_pass &= finite_epochs == epochs
            for axis, suffix in enumerate("xyz"):
                error_sum = sum(x["error_sum"][axis] for x in seeds)
                square_sum = sum(x["error_square_sum"][axis] for x in seeds)
                predicted = sum(x["predicted_variance_sum"][axis]
                                for x in seeds)/finite_epochs if finite_epochs else 0.0
                mean = error_sum/finite_epochs if finite_epochs else math.inf
                empirical = max(0.0, square_sum/finite_epochs - mean*mean) \
                    if finite_epochs else math.inf
                bias = abs(mean)/math.sqrt(predicted) if predicted > 0 else math.inf
                ratio = empirical/predicted if predicted > 0 else math.inf
                row[f"bias_over_sigma_{suffix}"] = bias
                row[f"empirical_predicted_variance_ratio_{suffix}"] = ratio
                covariance_pass &= bias < spec["gates"]["bias_over_predicted_sigma_max"]
                covariance_pass &= spec["gates"]["variance_ratio"][0] <= ratio <= \
                    spec["gates"]["variance_ratio"][1]
            nominal = key[1] == "regular" and int(key[5]) == -1
            row["small_noise_covariance_pass"] = covariance_pass and nominal
            if nominal:
                nominal_regular.append((row, sum(x["hmi"] for x in seeds),
                                        sum(x["model_valid"] for x in seeds)))
            writer.writerow(row)
            records.append(row)
    gates = spec["gates"]
    checks = {
        "nominal_regular_aggregate_count": len(nominal_regular) ==
            gates["nominal_regular_aggregate_count"],
        "nominal_covariance": all(row["small_noise_covariance_pass"]
                                  for row, _, _ in nominal_regular),
        "nominal_model_valid_no_operational_hmi": all(
            hmi == 0 for _, hmi, valid in nominal_regular if valid > 0),
    }
    result = {"status": PASS if all(checks.values()) else FAIL,
              "checks": checks, "aggregate_rows": len(records),
              "inventory": str(inventory_path), "summary": str(output_csv)}
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=pathlib.Path,
                        help="CSV shard or directory containing shards")
    parser.add_argument("output_csv", type=pathlib.Path)
    parser.add_argument("--protocol", type=pathlib.Path,
                        default=pathlib.Path("config/week4_validation.yaml"))
    parser.add_argument("--shard", action="append", type=pathlib.Path, default=[])
    parser.add_argument("--inventory", type=pathlib.Path)
    parser.add_argument("--result", type=pathlib.Path)
    parser.add_argument("--bootstrap", type=int, default=2000)
    args = parser.parse_args()
    inventory = args.inventory or args.output_csv.with_name("raw_inventory.json")
    result_path = args.result or args.output_csv.with_suffix(".json")
    try:
        result = summarize(discover(args.input, args.shard),
                           load_protocol(args.protocol), args.output_csv,
                           inventory, args.bootstrap)
    except InvalidArtifact as error:
        result = {"status": INVALID, "reason": str(error)}
    atomic_json(result_path, result)
    print(json.dumps(result, sort_keys=True))
    return 0 if result["status"] == PASS else (1 if result["status"] == FAIL else 2)


if __name__ == "__main__":
    raise SystemExit(main())
