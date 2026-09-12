#!/usr/bin/env python3
"""Post-seal obs_id-paired range recovery evaluator.

This executable is deliberately separate from the scientific estimator.  It
may read the locked clean pair and synthetic amplitude only after estimator
artifacts have been sealed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("percentile of empty data")
    position = (len(ordered) - 1) * q
    lo = math.floor(position)
    hi = math.ceil(position)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - position) + ordered[hi] * (position - lo)


def metrics(errors: list[float]) -> dict[str, float]:
    absolute = [abs(value) for value in errors]
    return {
        "mean_error_m": statistics.fmean(errors),
        "median_error_m": statistics.median(errors),
        "mae_m": statistics.fmean(absolute),
        "rmse_m": math.sqrt(statistics.fmean(value * value for value in errors)),
        "p95_absolute_error_m": percentile(absolute, 0.95),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--clean-observations", required=True, type=Path)
    parser.add_argument("--injected-observations", required=True, type=Path)
    parser.add_argument("--support", required=True, type=Path)
    parser.add_argument("--fixed-compensations", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--expected-count", required=True, type=int)
    parser.add_argument("--known-injection-amplitude-m", required=True, type=float)
    args = parser.parse_args()

    output = args.output_dir.resolve()
    if output.exists():
        raise ValueError("refusing to overwrite range evaluation directory")
    output.mkdir(parents=True)

    clean = {int(row["obs_id"]): row for row in read_csv(args.clean_observations)}
    injected = {
        int(row["obs_id"]): row for row in read_csv(args.injected_observations)
    }
    support = json.loads(args.support.read_text(encoding="utf-8"))
    segments = support.get("segments", [])
    compensation_rows = read_csv(args.fixed_compensations)
    compensation_by_segment = {
        row["segment_id"]: row for row in compensation_rows
    }
    candidate_pairs: list[dict[str, object]] = []
    for segment in segments:
        comp = compensation_by_segment.get(segment["segment_id"])
        if comp is None:
            raise ValueError("candidate segment has no fixed compensation")
        if comp.get("decision_use") not in {"1", "true", "True"} or \
                comp.get("final_use") not in {"1", "true", "True"}:
            raise ValueError("primary candidate segment was not actually recovered")
        delta = float(comp["delta_c_fixed_m"])
        if not math.isfinite(delta) or delta < 0.0:
            raise ValueError("fixed compensation is non-finite or negative")
        for obs_id_value in segment["obs_ids"]:
            obs_id = int(obs_id_value)
            if obs_id not in clean or obs_id not in injected:
                raise ValueError(f"obs_id pair missing: {obs_id}")
            clean_range = float(clean[obs_id]["raw_z_m"])
            injected_range = float(injected[obs_id]["raw_z_m"])
            recovered_range = injected_range - delta
            if recovered_range > injected_range + 1e-12:
                raise ValueError("positive-excess recovery moved in wrong direction")
            candidate_pairs.append({
                "obs_id": obs_id,
                "segment_id": segment["segment_id"],
                "tag_id": int(segment["tag_id"]),
                "anchor_id": int(segment["anchor_id"]),
                "z_clean_m": clean_range,
                "z_injected_m": injected_range,
                "delta_c_fixed_m": delta,
                "z_recovered_m": recovered_range,
                "raw_error_m": injected_range - clean_range,
                "recovered_error_m": recovered_range - clean_range,
            })
    if len(candidate_pairs) != args.expected_count:
        raise ValueError(
            f"affected pair count {len(candidate_pairs)} != {args.expected_count}")

    raw = metrics([float(row["raw_error_m"]) for row in candidate_pairs])
    recovered = metrics(
        [float(row["recovered_error_m"]) for row in candidate_pairs])
    deltas = [float(row["delta_c_fixed_m"]) for row in candidate_pairs]
    c_hats = [float(row["c_hat_stage2_m"]) for row in compensation_rows]
    if len(c_hats) != len(segments):
        raise ValueError("Stage2 amplitude/segment count mismatch")
    rmse_gain = raw["rmse_m"] - recovered["rmse_m"]
    mae_gain = raw["mae_m"] - recovered["mae_m"]
    summary = {
        "schema": "uifgo_pl_range_recovery_evaluation_v1",
        "status": "PASS" if recovered["rmse_m"] < raw["rmse_m"] - 1e-12
                  else "FAIL_RANGE_EFFECTIVENESS",
        "pairing": "EXACT_OBS_ID",
        "affected_pair_count": len(candidate_pairs),
        "raw": raw,
        "recovered": recovered,
        "improvement": {
            "rmse_m": rmse_gain,
            "rmse_pct": 100.0 * rmse_gain / raw["rmse_m"],
            "mae_m": mae_gain,
            "mae_pct": 100.0 * mae_gain / raw["mae_m"],
        },
        "compensation_m": {
            "mean": statistics.fmean(deltas),
            "median": statistics.median(deltas),
            "min": min(deltas),
            "max": max(deltas),
        },
        "diagnostic_only": {
            "known_injection_amplitude_m": args.known_injection_amplitude_m,
            "mean_stage2_c_hat_m": statistics.fmean(c_hats),
            "mean_stage2_c_hat_error_m":
                statistics.fmean(c_hats) - args.known_injection_amplitude_m,
            "known_amplitude_used_by_estimator": False,
            "clean_pair_used_by_estimator": False,
        },
    }
    fields = list(candidate_pairs[0])
    with (output / "range_recovery_evaluation.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(candidate_pairs)
    with (output / "range_recovery_summary.json").open(
            "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    return 0 if summary["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
